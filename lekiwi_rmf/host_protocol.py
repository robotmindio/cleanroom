"""Wire contract between the Pi motor host, its fake, and the ROS driver client.

Every key, command name and identifier rule that crosses the motor-host ZMQ
link is defined here once. The deployed Pi host speaks exactly this protocol;
changing any value is a protocol change that needs a version bump and a
coordinated host and driver deployment.
"""

from lekiwi_rmf.arm_trajectory import ARM_JOINTS


TELEMETRY_PROTOCOL_VERSION = 3
TELEMETRY_PROTOCOL_KEY = "_lekiwi_protocol"
TELEMETRY_SESSION_KEY = "_lekiwi_session"
TELEMETRY_SEQUENCE_KEY = "_lekiwi_sequence"
TELEMETRY_MONOTONIC_NS_KEY = "_lekiwi_sample_monotonic_ns"
TELEMETRY_TORQUE_ENABLED_KEY = "_lekiwi_torque_enabled"
TELEMETRY_KEYS = (
    TELEMETRY_PROTOCOL_KEY,
    TELEMETRY_SESSION_KEY,
    TELEMETRY_SEQUENCE_KEY,
    TELEMETRY_MONOTONIC_NS_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)
# LeRobot's LeKiwiClient rejects an observation without this camera manifest.
# The motor host serves no cameras, so it is always empty.
CAMERAS_KEY = "_cams"
HOST_ODOMETRY_KEY = "_lekiwi_odometry"
MOTOR_HEALTH_KEY = "_lekiwi_motor_health"
ARM_TRAJECTORY_STATUS_KEY = "_lekiwi_arm_trajectory"
# Optional action fields that renew the motor host's local arm-goal lease.
ARM_LEASE_KEYS = ("_lekiwi_arm_goal", "_lekiwi_arm_permission")
BASE_TEST_STAGE_KEY = "_lekiwi_base_test_stage"
BASE_TEST_STAGES = (.20, .25, .30)

BASE_VELOCITY_KEYS = ("x.vel", "y.vel", "theta.vel")
# Every observation carries these, and every motion action contains exactly these.
STATE_KEYS = tuple(f"{joint}.pos" for joint in ARM_JOINTS) + BASE_VELOCITY_KEYS
# LeRobot's LeKiwi bus motor names; motor health reports one ``servo/<name>`` each.
MOTOR_NAMES = ARM_JOINTS + ("base_left_wheel", "base_back_wheel", "base_right_wheel")


class TorqueCommand:
    """Request names accepted by the motor host's torque REP endpoint."""

    ENABLE = "enable"
    DISABLE = "disable"
    STATE = "state"
    TRAJECTORY_START = "trajectory_start"
    TRAJECTORY_CANCEL = "trajectory_cancel"


def valid_goal_id(value) -> bool:
    """Arm-goal ids are positive integers below 2**48, never booleans."""
    return type(value) is int and 0 < value < 2**48


def observation_payload(
    observation, *, session, sequence, sample_monotonic_ns,
    torque_enabled, motor_health, odometry, arm_status,
) -> dict:
    """Assemble one telemetry message from the motor-bus ``observation``.

    The key order is part of the deployed encoding: the message is serialized
    with plain ``json.dumps`` and sent as the only frame. The host carries no
    camera images.
    """
    return {
        CAMERAS_KEY: [],
        **observation,
        TELEMETRY_PROTOCOL_KEY: TELEMETRY_PROTOCOL_VERSION,
        TELEMETRY_SESSION_KEY: session,
        TELEMETRY_SEQUENCE_KEY: sequence,
        TELEMETRY_MONOTONIC_NS_KEY: sample_monotonic_ns,
        TELEMETRY_TORQUE_ENABLED_KEY: torque_enabled,
        MOTOR_HEALTH_KEY: motor_health,
        HOST_ODOMETRY_KEY: odometry,
        ARM_TRAJECTORY_STATUS_KEY: arm_status,
    }
