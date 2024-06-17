#!/usr/bin/env python3
"""Calculate desired velocities and depth based on the joystick input decided by the pilot
"""
from dataclasses import dataclass
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

global cmd_vel_publisher


@dataclass
class Param:
    """tuning and utils params"""

    vx_const: float = 0.07
    vy_const: float = 0.005
    wz_const: float = 1
    dz_const: float = 0.01
    pool_depth: float = 2  # will be 4 for the competition


PARAM = Param()


def map_from_to(
    num: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    """Mapping function

    Args:
        num (float): Value to be mapped
        in_min (float): Minimum value of input
        in_max (float): Maximum value of input
        out_min (float): Minimum value of ouput
        out_max (float): Maximum value of ouput

    Returns:
        float: Mapped value
    """
    return (
        float(num - in_min) / float(in_max - in_min) * (out_max - out_min)
    ) + out_min


def node_init():
    """Initialize motion_control node which subscribes to ROV/ctrl_sig topic and publishes
    on the ROV/cmd_vel topic during manual_control"""
    rospy.init_node("motion_control", anonymous=True)
    global cmd_vel_publisher
    cmd_vel_publisher = rospy.Publisher("ROV/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("ROV/line", Int16MultiArray, sig_recieved_callback)
    rospy.spin()


def sig_recieved_callback(msg):
    """callback function for the subscriber to the ROV/line_features topic

    Args:
        msg (Pose2D): line features vector as a Pose2D msg [x width theta]
    """
    thickness = msg.data[0]
    cx = msg.data[1]
    cy = msg.data[2]

    v_x = 17 - thickness 
    v_y = cx - 320
    d_z = cy - 250

    v_x = PARAM.vx_const * v_x  # desired v_x
    v_y = PARAM.vy_const * v_y  # desired v_y
    d_z = PARAM.dz_const * d_z  # desired w_z

    v_x = max(v_x,-0.35)
    v_x = min(v_x, 0.35)
    v_y = max(v_y,-0.45)
    v_y = min(v_y, 0.45)
    d_z = max(d_z,-1)
    d_z = min(d_z, 1)
    
    vel = Twist()
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0  # desired w_z
    vel.linear.x = v_x  # desired v_x
    vel.linear.y = v_y  # desired v_y
    vel.linear.z = d_z  # desired d_z

    rospy.loginfo(
        f"v_x: {vel.linear.x}, v_y: {vel.linear.y}, v_z: {vel.linear.z}, w_z: {vel.angular.z}"
    )
    cmd_vel_publisher.publish(vel)
    rospy.sleep(0.01)


if __name__ == "__main__":
    node_init()
