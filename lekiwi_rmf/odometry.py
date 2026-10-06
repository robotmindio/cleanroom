"""Pure telemetry and planar-odometry helpers used by the ROS driver."""

from dataclasses import dataclass
from math import cos, isfinite, sin

from lekiwi_rmf.launch_calibration import load_launch_calibration


TELEMETRY_PROTOCOL_VERSION = 2
TELEMETRY_PROTOCOL_KEY = "_lekiwi_protocol"
TELEMETRY_SESSION_KEY = "_lekiwi_session"
TELEMETRY_SEQUENCE_KEY = "_lekiwi_sequence"
TELEMETRY_MONOTONIC_NS_KEY = "_lekiwi_sample_monotonic_ns"
TELEMETRY_TORQUE_ENABLED_KEY = "_lekiwi_torque_enabled"
HOST_ODOMETRY_KEY = "_lekiwi_odometry"
BASE_XY_SCALE = 1.0
# 2026-10-05: source-time-aligned native lidar measured 1.08453 times the
# wheel yaw at the former 0.90 scale; independent RGB-D agreed on the turn.
BASE_YAW_SCALE = 0.976
TELEMETRY_KEYS = (
    TELEMETRY_PROTOCOL_KEY,
    TELEMETRY_SESSION_KEY,
    TELEMETRY_SEQUENCE_KEY,
    TELEMETRY_MONOTONIC_NS_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)


@dataclass(frozen=True)
class AcceptedTelemetry:
    """Identity and source timestamp of one validated observation packet."""

    token: tuple
    sample_monotonic_ns: int | None
    session_changed: bool = False
    torque_enabled: bool | None = None


def parse_telemetry_metadata(observation):
    """Parse the required repository protocol envelope."""
    if any(key not in observation for key in TELEMETRY_KEYS):
        raise ValueError("incomplete LeKiwi telemetry metadata")

    protocol = observation[TELEMETRY_PROTOCOL_KEY]
    session = observation[TELEMETRY_SESSION_KEY]
    sequence = observation[TELEMETRY_SEQUENCE_KEY]
    sample_ns = observation[TELEMETRY_MONOTONIC_NS_KEY]
    torque_enabled = observation[TELEMETRY_TORQUE_ENABLED_KEY]
    if isinstance(protocol, bool) or not isinstance(protocol, int):
        raise ValueError("telemetry protocol version must be an integer")
    if protocol != TELEMETRY_PROTOCOL_VERSION:
        raise ValueError(f"unsupported LeKiwi telemetry protocol {protocol}")
    if not isinstance(session, str) or not session or len(session) > 128:
        raise ValueError("telemetry session must be a non-empty string")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("telemetry sequence must be a non-negative integer")
    if isinstance(sample_ns, bool) or not isinstance(sample_ns, int) or sample_ns < 0:
        raise ValueError("telemetry sample timestamp must be a non-negative integer")
    if not isinstance(torque_enabled, bool):
        raise ValueError("telemetry torque state must be boolean")
    return session, sequence, sample_ns, torque_enabled


class TelemetrySequenceTracker:
    """Accept strictly ordered host samples after payload validation succeeds."""

    MAX_SESSIONS = 1024

    def __init__(self):
        self._session = None
        self._sequence = None
        self._sample_ns = None
        # Returning to any earlier host identity is replay, not a restart. Keep
        # all identities for this client lifetime; if a broken/hostile peer
        # churns identities indefinitely, fail closed instead of evicting old
        # replay protection.
        self._seen_sessions = set()

    def accept(self, observation):
        session, sequence, sample_ns, torque_enabled = parse_telemetry_metadata(observation)
        session_changed = self._session is not None and session != self._session
        if session_changed and session in self._seen_sessions:
            raise ValueError("retired telemetry session was replayed")
        if session not in self._seen_sessions:
            if len(self._seen_sessions) >= self.MAX_SESSIONS:
                raise ValueError("too many telemetry session changes")
            self._seen_sessions.add(session)
        if session == self._session:
            if sequence <= self._sequence:
                raise ValueError("duplicate or backward telemetry sequence")
            if sample_ns <= self._sample_ns:
                raise ValueError("non-monotonic telemetry sample timestamp")
        self._session = session
        self._sequence = sequence
        self._sample_ns = sample_ns
        return AcceptedTelemetry(
            ("host", session, sequence), sample_ns, session_changed, torque_enabled
        )


def accept_validated_telemetry(tracker, observation, required_state_keys):
    """Validate a decoded state payload before committing its freshness token."""
    missing = tuple(key for key in required_state_keys if key not in observation)
    if missing:
        raise ValueError(f"incomplete LeKiwi state: {', '.join(missing)}")
    try:
        values = (float(observation[key]) for key in required_state_keys)
        if not all(isfinite(value) for value in values):
            raise ValueError("non-finite LeKiwi state")
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("invalid LeKiwi state") from error
    return tracker.accept(observation)


class OdometrySampleClock:
    """Calculate intervals between accepted samples, never between timer ticks."""

    def __init__(self, max_interval=0.2):
        if max_interval <= 0:
            raise ValueError("maximum odometry interval must be positive")
        self.max_interval = float(max_interval)
        self.reset()

    def reset(self):
        self._token = None
        self._local_ns = None
        self._sample_ns = None
        self.discontinuity = None

    @staticmethod
    def _stream(token):
        return token[:2]

    def accept(self, token, local_monotonic_ns, sample_monotonic_ns=None):
        self.discontinuity = None
        previous_token = self._token
        previous_local_ns = self._local_ns
        previous_sample_ns = self._sample_ns
        self._token = token
        self._local_ns = int(local_monotonic_ns)
        self._sample_ns = sample_monotonic_ns

        if previous_token is None:
            return None
        if self._stream(token) != self._stream(previous_token):
            self.discontinuity = "telemetry source session changed"
            return None
        if sample_monotonic_ns is not None and previous_sample_ns is not None:
            elapsed_ns = sample_monotonic_ns - previous_sample_ns
        else:
            elapsed_ns = self._local_ns - previous_local_ns
        elapsed = elapsed_ns / 1e9
        if elapsed <= 0.0:
            self.discontinuity = "telemetry sample interval was not positive"
            return None
        if elapsed > self.max_interval:
            # The current sample becomes the new origin. Do not invent motion
            # across an implausible or unknown interval.
            self.discontinuity = (
                f"telemetry sample gap {elapsed:.3f}s exceeds {self.max_interval:.3f}s"
            )
            return None
        return elapsed


def integrate_pose(pose, velocity, dt):
    """Integrate body-frame x/y/yaw velocity into a world-frame 2D pose."""
    x, y, yaw = pose
    vx, vy, wz = velocity
    return (
        x + (vx * cos(yaw) - vy * sin(yaw)) * dt,
        y + (vx * sin(yaw) + vy * cos(yaw)) * dt,
        yaw + wz * dt,
    )


def load_base_scales(path):
    """Use the same saved wheel calibration as ros-start.sh."""
    values = load_launch_calibration(path)
    return (values.get("xy_velocity_scale", BASE_XY_SCALE),
            values.get("yaw_velocity_scale", BASE_YAW_SCALE))


def parse_host_odometry(payload):
    if HOST_ODOMETRY_KEY not in payload:
        return None  # Older/simulated hosts still use timestamped velocities.
    value = payload[HOST_ODOMETRY_KEY]
    if not isinstance(value, dict):
        raise ValueError("invalid host odometry")
    pose, scales, stamp = value.get("pose"), value.get("scales"), value.get("stamp_ns")
    if (not isinstance(pose, list) or len(pose) != 3
            or not isinstance(scales, list) or len(scales) != 2
            or any(type(v) not in (int, float) or not isfinite(v) for v in pose + scales)
            or any(v <= 0 for v in scales)
            or type(stamp) is not int or stamp <= 0):
        raise ValueError("invalid host odometry pose, calibration, or timestamp")
    return value


class HostOdometry:
    """Integrate measured velocity before transport can drop observations."""

    def __init__(self, xy_scale=BASE_XY_SCALE, yaw_scale=BASE_YAW_SCALE):
        self.scales = (xy_scale, yaw_scale)
        self.pose = (0.0, 0.0, 0.0)
        self.sample_ns = None
        self.velocity = None

    def update(self, velocity, sample_ns, stamp_ns):
        if (len(velocity) != 3 or not all(isfinite(v) for v in velocity)
                or type(sample_ns) is not int or sample_ns <= 0
                or type(stamp_ns) is not int or stamp_ns <= 0):
            raise ValueError("invalid local odometry sample")
        vx, vy, wz = velocity
        velocity = (vx * self.scales[0], vy * self.scales[0], wz * self.scales[1])
        if self.sample_ns is not None:
            dt = (sample_ns - self.sample_ns) / 1e9
            if dt <= 0:
                raise ValueError("host odometry clock did not advance")
            # Trapezoidal velocity and midpoint heading avoid the endpoint
            # bias during acceleration, braking, and simultaneous rotation.
            average = tuple((a + b) / 2 for a, b in zip(self.velocity, velocity))
            x, y, yaw = self.pose
            px, py, _ = integrate_pose((x, y, yaw + average[2] * dt / 2), average, dt)
            self.pose = (px, py, yaw + average[2] * dt)
        self.sample_ns, self.velocity = sample_ns, velocity
        return {"pose": list(self.pose), "scales": list(self.scales), "stamp_ns": stamp_ns}


class HostPoseTracker:
    """Align each host session to the driver's continuous local odom frame."""

    def __init__(self):
        self.session = None
        self.origin = None

    def update(self, session, host_pose, local_pose):
        hx, hy, ha = host_pose
        if session != self.session:
            x, y, yaw = local_pose
            rotation = yaw - ha
            self.origin = (x - hx * cos(rotation) + hy * sin(rotation),
                           y - hx * sin(rotation) - hy * cos(rotation), rotation)
            self.session = session
        x, y, rotation = self.origin
        return (x + hx * cos(rotation) - hy * sin(rotation),
                y + hx * sin(rotation) + hy * cos(rotation), rotation + ha)
