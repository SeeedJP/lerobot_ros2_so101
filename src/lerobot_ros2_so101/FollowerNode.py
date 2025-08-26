from .arm.FollowerArm import FollowerArm

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class FollowerNode(Node):
    def __init__(self):
        super().__init__("follower_node")

        self.declare_parameter("port", "")
        self.declare_parameter("joint_states", "/joint_states")
        self.declare_parameter("joint_commands", "/joint_commands")

        port = self.get_parameter("port").get_parameter_value().string_value
        joint_states = (
            self.get_parameter("joint_states").get_parameter_value().string_value
        )
        joint_commands = (
            self.get_parameter("joint_commands").get_parameter_value().string_value
        )

        self.follower_arm = FollowerArm(port=port)
        self.get_logger().info("Connect the follower arm.")
        self.follower_arm.connect()

        self.get_logger().info(f"Create publisher topic. {joint_states}")
        self.pub_joint_states = self.create_publisher(JointState, joint_states, 10)
        self.get_logger().info(f"Create subscription topic. {joint_commands}")
        self.sub_joint_commands = self.create_subscription(
            JointState, joint_commands, self.joint_commands_callback, 10
        )

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        positions = self.follower_arm.read_present_position()
        positions = {k: (v - 2048) * (2 * 3.14) / 4096 for k, v in positions.items()}

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        msg.velocity = []
        msg.effort = []
        self.pub_joint_states.publish(msg)

    def joint_commands_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        positions = {k: int(v * 4096 / (2 * 3.14) + 2048) for k, v in positions.items()}

        self.follower_arm.write_goal_position(positions)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
