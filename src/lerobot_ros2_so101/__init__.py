import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode, TorqueMode
from lerobot.motors import Motor, MotorNormMode

def TestLerobot():
    norm_mode_body = MotorNormMode.RANGE_M100_100

    bus = FeetechMotorsBus(
        port="/dev/ttyACM0",
        motors={
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
    )

    bus.connect()

    # SO101Leader.configure()
    ## bus.disable_torque()
    for motor in bus.motors:
        bus.write("Torque_Enable", motor, TorqueMode.DISABLED.value, num_retry=0)
        bus.write("Lock", motor, 0, num_retry=0)
    bus.configure_motors()
    for motor in bus.motors:
        bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    # SO101Leader.calibrate()
    # bus.disable_torque()
    # for motor in bus.motors:
    #     bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
    homing_offsets = bus.set_half_turn_homings()
    range_mins, range_maxes = bus.record_ranges_of_motion()

    bus.disconnect()

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    TestLerobot()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

