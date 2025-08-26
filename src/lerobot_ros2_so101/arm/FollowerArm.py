from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.motors_bus import MotorCalibration
from lerobot.motors import Motor, MotorNormMode

import draccus


class FollowerArm:
    def __init__(self, port=None):
        self.port = port

        # Load calibration file
        with (
            open(
                "/home/jetson/.cache/huggingface/lerobot/calibration/robots/so101_follower/lerobot_follower.json"
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

        # SO101Follower.configure()
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", motor, 16)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)

    def read_present_position(self) -> dict[str, float]:
        return self.bus.sync_read("Present_Position", normalize=False)

    def write_goal_position(self, positions) -> None:
        missing = [k for k in positions if k not in self.bus.motors]
        if missing:
            print(f"WARNING: motors not defined. {missing}")
            return
        for k, v in positions.items():
            self.bus.write("Goal_Position", k, v, normalize=False)
