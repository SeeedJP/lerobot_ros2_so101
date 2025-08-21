# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# LeRobot
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode, TorqueMode
from lerobot.motors.motors_bus import MotorCalibration
from lerobot.motors import Motor, MotorNormMode

import draccus


class LeaderArm:
    def __init__(self):
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
            port="/dev/ttyACM0",
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
        action = self.bus.sync_read("Present_Position", normalize=False)
        return action


class MinimalPublisher(Node):
    def __init__(self, leader_arm):
        super().__init__("minimal_publisher")
        self.leader_arm = leader_arm
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        positions = self.leader_arm.read_present_position()
        positions = {k: (v - 2048) * (2 * 3.14) / 4096 for k, v in positions.items()}
        print(positions)

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)


def main(args=None):
    leader_arm = LeaderArm()
    leader_arm.connect()

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(leader_arm)
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()
