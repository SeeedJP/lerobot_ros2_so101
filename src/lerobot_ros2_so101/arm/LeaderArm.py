from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.motors_bus import MotorCalibration
from lerobot.motors import Motor, MotorNormMode

import draccus


class LeaderArm:
    def __init__(self, port=None):
        self.port = port

        # Load calibration file
        with (
            open(
                "/home/jetson/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/lerobot_leader.json"
            ) as f,
            draccus.config_type("json"),
        ):
            self.calibration = draccus.load(dict[str, MotorCalibration], f)

    def connect(self) -> None:
        self.bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_0_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_0_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_0_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_0_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_0_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )

        self.bus.connect()

        # SO101Leader.configure()
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def read_present_position(self) -> dict[str, float]:
        # SO101Leader.get_action()
        return self.bus.sync_read("Present_Position", normalize=False)
