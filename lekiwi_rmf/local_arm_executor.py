"""Motor-host trajectory execution; clocks and feedback never cross the network."""

import math
from copy import deepcopy

from lekiwi_rmf.arm_trajectory import (
    ARM_JOINTS, JOINT_LIMITS, action_positions, joint_positions,
    prepare_trajectory, sample_trajectory,
)
from lekiwi_rmf.host_protocol import valid_goal_id


def validate_status(value):
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != {"id", "state", "elapsed", "code", "detail"}:
        raise ValueError("invalid local arm status")
    if (not valid_goal_id(value["id"])
            or value["state"] not in {"waiting", "running", "paused", "succeeded", "aborted", "canceled"}
            or type(value["elapsed"]) not in (int, float)
            or not math.isfinite(value["elapsed"]) or value["elapsed"] < 0
            or type(value["code"]) is not int or value["code"] not in {0, -1, -4, -5}
            or not isinstance(value["detail"], str)):
        raise ValueError("invalid local arm status")
    return value


class LocalArmExecutor:
    """One retained goal, bounded safety lease, and an indefinite measured hold."""

    def __init__(self, clock, lease_timeout):
        self.clock = clock
        self.lease_timeout = lease_timeout
        self.goal = None
        self.status = None
        self.hold = {}
        self.lease_at = None
        self.last_tick = clock()

    @property
    def active(self):
        return self.status is not None and self.status["state"] in {"waiting", "running", "paused"}

    def start(self, request, observation):
        if not isinstance(request, dict):
            raise ValueError("arm trajectory must be an object")
        request = deepcopy(request)
        goal_id = request.get("id")
        if not valid_goal_id(goal_id):
            raise ValueError("invalid arm goal id")
        if self.status is not None and self.status["id"] == goal_id:
            if request != self.goal["request"]:
                raise ValueError("arm goal id reused with different content")
            return self.status  # Idempotent upload after a lost acknowledgment.
        zeros, directions = request["zeros"], request["directions"]
        if (set(zeros) != set(ARM_JOINTS) or set(directions) != set(ARM_JOINTS)
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in zeros.values())
                or any(type(v) not in (int, float) or v not in (-1, 1) for v in directions.values())):
            raise ValueError("invalid arm calibration")
        names, rows = request["names"], request["points"]
        if not isinstance(rows, list) or not 1 <= len(rows) <= 10000:
            raise ValueError("arm trajectory must have 1..10000 points")
        self._validate_observation(observation)
        actual = joint_positions(observation, zeros, directions)
        points = prepare_trajectory(names, rows, actual, zeros, directions)
        for key in ("path", "goal"):
            tolerances = request[key]
            if not set(tolerances) <= set(names) or any(
                type(v) not in (int, float) or not math.isfinite(v) or v <= 0
                for v in tolerances.values()
            ):
                raise ValueError("invalid trajectory tolerances")
        settling, delay = request["settling"], request["delay"]
        if (type(settling) not in (int, float) or not math.isfinite(settling) or not 0 < settling <= 60
                or type(delay) not in (int, float) or not math.isfinite(delay) or not 0 <= delay <= 2):
            raise ValueError("invalid arm settling window or start delay")
        self.goal = {"request": request, "points": points, "start": actual, "elapsed": -delay}
        self.status = {"id": goal_id, "state": "waiting", "elapsed": 0.0, "code": 0, "detail": ""}
        self.hold = {f"{name}.pos": observation[f"{name}.pos"] for name in ARM_JOINTS}
        self.lease_at = None
        self.last_tick = self.clock()
        return self.status

    def renew(self, goal_id, permitted):
        if self.status is None or goal_id != self.status["id"]:
            return
        self.lease_at = self.clock() if permitted else None

    def cancel(self, goal_id=None):
        if self.status is not None and (goal_id is None or goal_id == self.status["id"]):
            if self.active:
                self.status.update(state="canceled", code=-1, detail="arm goal canceled")
            self.lease_at = None

    @staticmethod
    def _validate_observation(observation):
        if not isinstance(observation, dict) or any(
            type(observation.get(f"{name}.pos")) not in (int, float)
            or not math.isfinite(observation[f"{name}.pos"])
            for name in ARM_JOINTS
        ):
            raise ValueError("local arm feedback is incomplete or non-finite")

    def step(self, observation, feedback_at):
        now = self.clock()
        dt, self.last_tick = max(0.0, now - self.last_tick), now
        if not self.active:
            return None
        # A delayed host loop, stale bus read, or missing remote safety decision
        # pauses the local clock. No elapsed gap is replayed when traffic returns.
        permitted = self.lease_at is not None and 0 <= now - self.lease_at <= self.lease_timeout
        fresh = feedback_at is not None and 0 <= now - feedback_at <= 0.25
        if fresh:
            self._validate_observation(observation)
        if not permitted or not fresh or dt > 0.15:
            entering_pause = self.status["state"] != "paused"
            self.status["state"] = "paused"
            if fresh and entering_pause:
                self.hold = {f"{name}.pos": observation[f"{name}.pos"] for name in ARM_JOINTS}
            return {**self.hold, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        if self.status["state"] in {"paused", "waiting"}:
            dt = 0.0
        self.status["state"] = "running"
        self.goal["elapsed"] += dt
        elapsed = max(0.0, self.goal["elapsed"])
        self.status["elapsed"] = elapsed
        request = self.goal["request"]
        actual = joint_positions(observation, request["zeros"], request["directions"])
        positions, _, _ = sample_trajectory(
            request["names"], self.goal["start"], self.goal["points"], elapsed,
        )
        final = self.goal["points"][-1]
        violation = next((name for name, limit in request["path"].items()
                          if elapsed < final.time and abs(actual[name] - positions[name]) > limit), None)
        if violation:
            self.status.update(state="aborted", code=-4, detail=(
                f"path tolerance exceeded for {violation}: "
                f"error={abs(actual[violation] - positions[violation]):.4f} rad, "
                f"limit={request['path'][violation]:.4f} rad, elapsed={elapsed:.3f} s"
            ))
        elif elapsed >= final.time and all(
            abs(actual[name] - final.positions[name]) <= limit for name, limit in request["goal"].items()
        ):
            self.status["state"] = "succeeded"
        elif elapsed > final.time + request["settling"]:
            worst = max(request["goal"], key=lambda name: abs(actual[name] - final.positions[name]) / request["goal"][name])
            self.status.update(state="aborted", code=-5, detail=(
                f"goal tolerance exceeded for {worst}: error={abs(actual[worst] - final.positions[worst]):.4f} rad, "
                f"limit={request['goal'][worst]:.4f} rad"
            ))
        if self.status["state"] == "aborted":
            self.hold = {f"{name}.pos": observation[f"{name}.pos"] for name in ARM_JOINTS}
        else:
            # A measured start beyond a joint bound may recover inward; every setpoint stays bounded.
            bounded = {name: min(JOINT_LIMITS[name][1], max(JOINT_LIMITS[name][0], value))
                       for name, value in positions.items()}
            self.hold.update({f"{name}.pos": value for name, value in action_positions(
                bounded.keys(), bounded.values(), request["zeros"], request["directions"],
            ).items()})
        return {**self.hold, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
