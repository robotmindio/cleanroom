"""Motion permission policy independent of ROS messages and executors."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum

from lekiwi_rmf.motion_guards import FUTURE_STAMP_TOLERANCE_NS


class SafetyState(str, Enum):
    BOOT = "BOOT"
    DISARMED = "DISARMED"
    READY = "READY"
    ARMED = "ARMED"
    FAULT_LATCHED = "FAULT_LATCHED"
    ESTOP = "ESTOP"


@dataclass(frozen=True)
class Requirement:
    max_age_ns: int
    base: bool = True
    arm: bool = True
    # An invalid frame counts as a dropped frame: the last healthy sample
    # keeps its lease until max_age_ns, so isolated bad frames do not
    # interrupt motion, while a sustained blind sensor still faults.
    tolerate_invalid: bool = False


@dataclass
class InputSample:
    healthy: bool
    stamp_ns: int
    detail: str = ""


@dataclass
class SafetyDecision:
    state: SafetyState
    base_permitted: bool
    arm_permitted: bool
    faults: tuple[str, ...] = ()
    latched_faults: tuple[str, ...] = ()
    arm_workspace_clear: bool = True
    arm_stowed: bool = False


@dataclass
class SafetyStateMachine:
    """Default-deny health evaluator. Faults latch only when latch_faults is set.

    Without latching, permission is withdrawn while an input is unhealthy and returns
    by itself when it recovers. An e-stop always latches: it is an operator action.
    """

    requirements: dict[str, Requirement]
    samples: dict[str, InputSample] = field(default_factory=dict)
    driver_state: str = "DISARMED"
    arm_stowed: bool = False
    fault_latched: bool = False
    estop_latched: bool = False
    latch_faults: bool = True
    base_ever_ready: bool = False
    arm_ever_ready: bool = False
    latched_faults: list[str] = field(default_factory=list)
    driver_state_required: bool = True
    # The Pi's scan clock was measured 7 ms ahead of compute; bound the grace.
    future_stamp_tolerance_ns: int = FUTURE_STAMP_TOLERANCE_NS

    def _latch(self, faults: tuple[str, ...]) -> None:
        self.fault_latched = True
        self.latched_faults.extend(
            fault for fault in faults if fault not in self.latched_faults
        )

    def update(self, name: str, healthy: bool, stamp_ns: int, detail: str = "") -> None:
        if name not in self.requirements:
            return
        requirement = self.requirements[name]
        previous = self.samples.get(name)
        if (
            not healthy and requirement.tolerate_invalid and previous is not None
            and previous.healthy and 0 <= stamp_ns - previous.stamp_ns <= requirement.max_age_ns
        ):
            return
        self.samples[name] = InputSample(bool(healthy), int(stamp_ns), detail)
        if name == "estop" and not healthy:
            self.estop_latched = True
        elif not healthy:
            if self.latch_faults and (
                (requirement.base and self.base_ever_ready)
                or (requirement.arm and self.arm_ever_ready)
            ):
                self._latch((f"{name}: {detail or 'unhealthy'}",))

    def _faults(self, now_ns: int, for_base: bool, for_arm: bool) -> tuple[str, ...]:
        faults: list[str] = []
        for name, requirement in self.requirements.items():
            if not ((for_base and requirement.base) or (for_arm and requirement.arm)):
                continue
            fault = self._requirement_fault(name, requirement, now_ns)
            if fault is not None:
                faults.append(fault)
        return tuple(faults)

    def _requirement_fault(self, name: str, requirement: Requirement, now_ns: int) -> str | None:
        sample = self.samples.get(name)
        if sample is None:
            return f"{name}: missing"
        age = now_ns - sample.stamp_ns
        if age < -self.future_stamp_tolerance_ns or age > requirement.max_age_ns:
            return f"{name}: stale"
        if not sample.healthy:
            return f"{name}: {sample.detail or 'unhealthy'}"
        return None

    def decision(self, now_ns: int) -> SafetyDecision:
        base_faults = self._faults(now_ns, True, False)
        arm_faults = self._faults(now_ns, False, True)
        workspace_requirement = self.requirements.get("arm_workspace")
        arm_workspace_clear = (
            workspace_requirement is None
            or self._requirement_fault("arm_workspace", workspace_requirement, now_ns) is None
        )
        joints_requirement = self.requirements.get("joints")
        arm_stowed = self.arm_stowed and (
            joints_requirement is None
            or self._requirement_fault("joints", joints_requirement, now_ns) is None
        )
        all_faults = tuple(dict.fromkeys((*base_faults, *arm_faults)))
        if self.driver_state == "LINK_LOST":
            all_faults = (*all_faults, "driver: link lost")
            base_faults = (*base_faults, "driver: link lost")
            arm_faults = (*arm_faults, "driver: link lost")
            if self.latch_faults and (self.base_ever_ready or self.arm_ever_ready):
                self._latch(("driver: link lost",))
        elif self.latch_faults and (
            (base_faults and self.base_ever_ready)
            or (arm_faults and self.arm_ever_ready)
        ):
            # A dependency becoming stale after its scope was ready is a
            # global runtime fault, not a return to partial startup state.
            faults = (
                (base_faults if self.base_ever_ready else ())
                + (arm_faults if self.arm_ever_ready else ())
            )
            self._latch(faults)

        if self.estop_latched:
            return SafetyDecision(
                SafetyState.ESTOP, False, False, all_faults, tuple(self.latched_faults),
                arm_workspace_clear, arm_stowed,
            )
        if self.fault_latched:
            return SafetyDecision(
                SafetyState.FAULT_LATCHED, False, False, all_faults, tuple(self.latched_faults),
                arm_workspace_clear, arm_stowed,
            )

        base_ready = not base_faults
        arm_ready = not arm_faults
        if not base_ready and not arm_ready:
            return SafetyDecision(
                SafetyState.BOOT, False, False, all_faults,
                arm_workspace_clear=arm_workspace_clear, arm_stowed=arm_stowed,
            )

        self.base_ever_ready = self.base_ever_ready or base_ready
        self.arm_ever_ready = self.arm_ever_ready or arm_ready
        armed = self.driver_state == "ARMED"
        base_permitted = armed and base_ready and arm_stowed
        arm_permitted = armed and arm_ready
        if armed:
            return SafetyDecision(
                SafetyState.ARMED, base_permitted, arm_permitted, all_faults,
                arm_workspace_clear=arm_workspace_clear, arm_stowed=arm_stowed,
            )
        # In READY these are short-lived capability-readiness leases, not
        # evidence of motion or torque. The disarmed driver still rejects all
        # commands, but the explicit arm transaction requires the arm-specific
        # lease because enabling torque can move a loaded arm.
        return SafetyDecision(
            SafetyState.READY,
            base_ready and arm_stowed,
            arm_ready,
            all_faults,
            arm_workspace_clear=arm_workspace_clear, arm_stowed=arm_stowed,
        )

    def reset(self, now_ns: int) -> tuple[bool, str]:
        if self.driver_state_required and self.driver_state == "ARMED":
            return False, "disarm the driver before resetting safety faults"
        faults = self._faults(now_ns, True, True)
        if faults:
            return False, "; ".join(faults)
        self.fault_latched = False
        self.estop_latched = False
        self.latched_faults.clear()
        self.base_ever_ready = True
        self.arm_ever_ready = True
        if self.driver_state_required:
            return True, "safety fault reset; robot remains disarmed"
        return True, "safety fault reset; no hardware driver state is configured"


def permit_unless_strict(
    decision: SafetyDecision, strict: bool, driver_state: str = ""
) -> SafetyDecision:
    """Keep capability health gates active in the recoverable normal mode.

    Holding an existing arm goal through feedback gaps belongs to the driver;
    it must not grant new arm movement through motor-health errors here.
    """
    if strict or decision.state in {SafetyState.ESTOP, SafetyState.FAULT_LATCHED}:
        return decision
    state = SafetyState.ARMED if driver_state == "ARMED" else decision.state
    return replace(decision, state=state)


