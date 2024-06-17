from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


@dataclass
class Param:
    """tuning and utils params"""

    vx_const: float = 1.3
    vy_const: float = 1.3
    wz_const: float = 0.8
    dz_const: float = 1
    pool_depth: float = 2.0  # will be 4 for the competition


PARAM = Param()


class MotionControl(Node):
    def __init__(self):
        super().__init__("motion_control")
        self.cmd_vel_publisher = self.create_publisher(Twist, "ROV/cmd_vel", 10)
        self.create_subscription(
            Twist, "ROV/joystick", self.ctrl_sig_recieved_callback, 10
        )

    def ctrl_sig_recieved_callback(self, twist_msg: Twist):
        """callback function for the subscriber to the ROV/joystick topic

        Args:
            msg (Twist): Twist message sent by the joystick
        """
        v_x = twist_msg.linear.x
        v_y = twist_msg.linear.y
        d_z = twist_msg.linear.z
        w_z = twist_msg.angular.z

        v_x = PARAM.vx_const * v_x  # desired v_x
        v_y = PARAM.vy_const * v_y  # desired v_y
        # d_z = PARAM.pool_depth * d_z  # desired d_z
        # d_z = map_from_to(d_z, -1, 1, 1, -1)  # desired d_z
        d_z = PARAM.dz_const * d_z
        w_z = PARAM.wz_const * w_z  # desired w_z

        vel = Twist()
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = w_z  # desired w_z
        vel.linear.x = v_x  # desired v_x
        vel.linear.y = v_y  # desired v_y
        vel.linear.z = d_z   # desired d_z

        self.cmd_vel_publisher.publish(vel)


def main(args=None):
    rclpy.init(args=args)

    motion_control_node = MotionControl()
    rclpy.spin(motion_control_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
