import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from PySticks.pysticks import get_controller


class JoyStickNode(Node):
    def __init__(self):
        super().__init__("joystick")

        self.joystick_publisher = self.create_publisher(Twist, "ROV/joystick", 10)
        self.gripper_publisher = self.create_publisher(Bool, "ROV/gripper", 1)

        self.controller = get_controller()

        self.twist_msg = Twist()
        self.gripper_msg = Bool()

    def update(self):
        self.controller.update()

        aim_ball_x = self.controller.getAimball()[0]
        aim_ball_y = self.controller.getAimball()[1]

        # Get depth value readings
        self.controller.depthUpFine()
        self.controller.depthDownFine()
        
        self.twist_msg.linear.x = self.controller.getPitch() if not aim_ball_x else 0
        self.twist_msg.linear.y = self.controller.getRoll() if not aim_ball_y else 0
        self.twist_msg.linear.z = self.controller.depth
        self.twist_msg.angular.z = self.controller.getYaw() if not aim_ball_x and not aim_ball_y else 0

        self.gripper_msg.data = bool(self.controller.getTrigger() + 1)

        self.get_logger().info("Throttle: %+2.2f   Roll: %+2.2f   Pitch: %+2.2f   Yaw: %+2.2f   Aux: %+2.2f"
            % (
                self.controller.depth,
                self.controller.getRoll(),
                self.controller.getPitch(),
                self.controller.getYaw(),
                bool(self.controller.getTrigger() + 1),
            ))

        self.gripper_publisher.publish(self.gripper_msg)

        if self.controller.stopAll():
            self.twist_msg = Twist()

        self.joystick_publisher.publish(self.twist_msg)




def main(args=None):
    rclpy.init(args=args)

    jotstick_node = JoyStickNode()

    while rclpy.ok():
        jotstick_node.update()

        rclpy.spin(jotstick_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()

