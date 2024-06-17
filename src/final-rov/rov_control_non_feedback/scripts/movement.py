#!/usr/bin/env python3

"""Callibration of all the parameters that could be tuned
        the tuning could be sent via the joystick buttons or the ros topic pup
"""
import dataclasses
from time import time
import math
from typing import List
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from rov_control.cfg import pidConfig

@dataclasses.dataclass
class ErrorVal:
    """PID error terms"""

    e_sum: float = 0
    d_error: float = 0
    current_error: float = 0
    prev_error: float = 0


class PID:
    """PID controller Class"""

    def __init__(
        self,
        k_proportaiol: float,
        k_integral: float,
        k_derivative: float,
        windup_val: float,
    ) -> None:
        """Creates a PID controller using provided PID parameters

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative
        self.pid: float = 0
        self.error = ErrorVal()
        self.windup_val = windup_val

    def compute(self, ref: float, measured: float) -> float:
        """Computes the PID value based on the given reference and measured output values

        Args:
            ref (float): reference signal that we desire to track
            measured (float): actual measured output of the signal

        Returns:
            float: PID value
        """
        self.error.current_error = ref - measured
        self.error.e_sum += self.error.current_error
        self.error.d_error = self.error.current_error - self.error.prev_error
        self.pid = (
            self.k_proportaiol * self.error.current_error
            + self.k_integral * self.error.e_sum
            + self.k_derivative * self.error.d_error
        )
        self.error.prev_error = self.error.current_error
        if self.error.current_error <= self.windup_val:
            self.error.e_sum = 0
        return self.pid

    def set_pid(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Sets the PID controller constants

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative


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


@dataclasses.dataclass
class Param:
    """Tuning and utils params"""

    kp_x = 1
    ki_x = 0
    kd_x = 0

    kp_y = 1
    ki_y = 0
    kd_y = 0

    kp_w = 1
    ki_w = 0
    kd_w = 0

    kp_depth = 1
    ki_depth = 0
    kd_depth = 0

    kp_roll = 1
    ki_roll = 0
    kd_roll = 0

    windup_val_x = 0.1
    windup_val_y = 0.1
    windup_val_w = 0.1
    windup_val_depth = 0.1
    windup_val_roll = 0.1

    floatability = 0  # TODO: measure the floatability of the ROV
    roll_desired = 0  # TODO: check the roll stable angle
    thruster_max = 1792
    thruster_min = 1180  # Changed to 1180 instead of 1160 to make sure that the center is 1485 which is a stoping value

    # corredponding to motion control node
    # TODO: adjust the PARAM.min and PARAM.max to all 3 motions
    max_vx = 3
    min_vx = -3
    max_vy = 3
    min_vy = -3
    max_wz = 0.7
    min_wz = -0.7
    max_pool_depth = 1
    max_roll = 90
    min_roll = -90

    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0
    angular_offset = 0


@dataclasses.dataclass
class Measured:
    """Sensor measurments"""

    v_x = 0
    v_y = 0
    w_z = 0
    roll = 0
    depth = 0


@dataclasses.dataclass
class Time:
    """Time related variables for integration and differentiation"""

    t_prev = 0
    t_current = 0
    delta_t = 0


PARAM = Param()
actual = Measured()
t = Time()
pid_vx = PID(
    k_proportaiol=PARAM.kp_x,
    k_derivative=PARAM.kd_x,
    k_integral=PARAM.ki_x,
    windup_val=PARAM.windup_val_x,
)
pid_vy = PID(
    k_proportaiol=PARAM.kp_y,
    k_derivative=PARAM.kd_y,
    k_integral=PARAM.ki_y,
    windup_val=PARAM.windup_val_y,
)
pid_wz = PID(
    k_proportaiol=PARAM.kp_w,
    k_derivative=PARAM.kd_w,
    k_integral=PARAM.ki_w,
    windup_val=PARAM.windup_val_w,
)
pid_depth = PID(
    k_proportaiol=PARAM.kp_depth,
    k_derivative=PARAM.kd_depth,
    k_integral=PARAM.ki_depth,
    windup_val=PARAM.windup_val_depth,
)
pid_stabilization = PID(
    k_proportaiol=PARAM.kp_roll,
    k_derivative=PARAM.kd_roll,
    k_integral=PARAM.ki_roll,
    windup_val=PARAM.windup_val_roll,
)

global thrusters_voltages_publisher


def node_init():
    """Initialize motion_control node which subscribes to ROV/cmd_vel topic and the ROV/depth
    and publishes on the ROV/thrusters topic"""
    rospy.init_node("kinematic_model", anonymous=True)
    
    rospy.on_shutdown(stop_all) # stop all thrusters

    global thrusters_voltages_publisher

    thrusters_voltages_publisher = rospy.Publisher(
        "ROV/thrusters", Float32MultiArray, queue_size=10
    )

    # subscribers for communicating with other subsystems of the ROV to get the measurements of
    # depth, imu (linear and angular velocities), and desired velocities and depth from the joystick
    rospy.Subscriber("ROV/cmd_vel", Twist, cmd_vel_recieved_callback)
    rospy.Subscriber("ROV/depth", Float64, depth_recieved_callback)
    rospy.Subscriber("ROV/imu", Imu, imu_recieved_callback)

    srv = Server(pidConfig, set_param_callback)
    
    PARAM.kp_depth = rospy.get_param("/pid/depth/kp")
    PARAM.ki_depth = rospy.get_param("/pid/depth/ki")
    PARAM.kd_depth = rospy.get_param("/pid/depth/kd")
    PARAM.kp_x = rospy.get_param("/pid/x/kp")
    PARAM.ki_x = rospy.get_param("/pid/x/ki")
    PARAM.kd_x = rospy.get_param("/pid/x/kd")
    PARAM.kp_y = rospy.get_param("/pid/y/kp")
    PARAM.ki_y = rospy.get_param("/pid/y/ki")
    PARAM.kd_y = rospy.get_param("/pid/y/kd")
    PARAM.kp_w = rospy.get_param("/pid/w/kp")
    PARAM.ki_w = rospy.get_param("/pid/w/ki")
    PARAM.kd_w = rospy.get_param("/pid/w/kd")
    PARAM.kp_roll = rospy.get_param("/pid/roll/kp")
    PARAM.ki_roll = rospy.get_param("/pid/roll/ki")
    PARAM.kd_roll = rospy.get_param("/pid/roll/kd")
    PARAM.floatability = rospy.get_param("/pid/floatability")

    # PARAM.ki_depth = pid_prm.y
    # PARAM.kd_depth = pid_prm.z

    #################################################################################################
    # subscriber for tuning the parameters through the rqt
    # rospy.Subscriber("ROV/PID", Vector3, pid_depth_calibration_callback)
    # rospy.Subscriber("ROV/PID", Vector3, pid_x_calibration_callback)
    # rospy.Subscriber("ROV/PID", Vector3, pid_y_calibration_callback)
    # rospy.Subscriber("ROV/PID", Vector3, pid_w_calibration_callback)
    # rospy.Subscriber("ROV/op", Vector3, op_calibration_callback)
    rospy.spin()


# def pid_depth_calibration_callback(pid_prm):
#     """subscriber callback for tuning PID parameters through the rqt

#     Args:
#         pid_prm (Vector3): A Vector3 message containing kp in x, ki in y, and kd in z
#     """
#     PARAM.kp_depth = pid_prm.x
#     PARAM.ki_depth = pid_prm.y
#     PARAM.kd_depth = pid_prm.z


# def pid_x_calibration_callback(pid_prm):
#     """subscriber callback for tuning x velocity PID parameters through the rqt

#     Args:
#         pid_prm (Vector3): A Vector3 message containing kp in x, ki in y, and kd in z
#     """
#     PARAM.kp_x = pid_prm.x
#     PARAM.ki_x = pid_prm.y
#     PARAM.kd_x = pid_prm.z


# def pid_y_calibration_callback(pid_prm):
#     """subscriber callback for tuning y velocity PID parameters through the rqt

#     Args:
#         pid_prm (Vector3): A Vector3 message containing kp in x, ki in y, and kd in z
#     """
#     PARAM.kp_y = pid_prm.x
#     PARAM.ki_y = pid_prm.y
#     PARAM.kd_y = pid_prm.z


# def pid_w_calibration_callback(pid_prm):
#     """subscriber callback for tuning angular velocity PID parameters through the rqt

#     Args:
#         pid_prm (Vector3): A Vector3 message containing kp in x, ki in y, and kd in z
#     """
#     PARAM.kp_w = pid_prm.x
#     PARAM.ki_w = pid_prm.y
#     PARAM.kd_w = pid_prm.z


# def op_calibration_callback(op_prm):
#     """subscriber callback for tuning other parameters through the rqt

#     Args:
#         op_prm (_type_): A Vector3 message containing floatability in x, and depth value in y
#     """
#     PARAM.floatability = op_prm.x


#################################################################################################

def set_param_callback(config,level):
    if not level:
        config['kp_depth'] = PARAM.kp_depth
        config['ki_depth'] = PARAM.ki_depth
        config['kd_depth'] = PARAM.kd_depth
        config['kp_x'] = PARAM.kp_x
        config['ki_x'] = PARAM.ki_x
        config['kd_x'] = PARAM.kd_x
        config['kp_y'] = PARAM.kp_y
        config['ki_y'] = PARAM.ki_y
        config['kd_y'] = PARAM.kd_y
        config['kp_w'] = PARAM.kp_w
        config['ki_w'] = PARAM.ki_w
        config['kd_w'] = PARAM.kd_w
        config['kp_roll'] = PARAM.kp_roll
        config['ki_roll'] = PARAM.ki_roll
        config['kd_roll'] = PARAM.kd_roll
        config['floatability'] = PARAM.floatability
        return config
    PARAM.kp_depth = config['kp_depth']
    PARAM.ki_depth = config['ki_depth']
    PARAM.kd_depth = config['kd_depth']
    PARAM.kp_x = config['kp_x']
    PARAM.ki_x = config['ki_x']
    PARAM.kd_x = config['kd_x']
    PARAM.kp_y = config['kp_y']
    PARAM.ki_y = config['ki_y']
    PARAM.kd_y = config['kd_y']
    PARAM.kp_w = config['kp_w']
    PARAM.ki_w = config['ki_w']
    PARAM.kd_w = config['kd_w']
    PARAM.kp_roll = config['kp_roll']
    PARAM.ki_roll = config['ki_roll']
    PARAM.kd_roll = config['kd_roll']
    PARAM.floatability = config['floatability']
    return config

def depth_recieved_callback(depth):
    """callback function for the subscriber to the ROV/depth topic

    Args:
        msg (Float64): depth sent by the the depth sensor
    """
    actual.depth = depth
    rospy.loginfo(f"depth: {depth}")


def imu_recieved_callback(imu):
    """recieves the imu readings from the MPU9025
    Args:
        imu (sensor_msgs/Imu): sensor message containing linear accelerations and angular velocities
    """
    _roll_dot, pitch_dot, _yaw_dot = [
        imu.angular_velocity.x,
        imu.angular_velocity.y,
        imu.angular_velocity.z,
    ]
    roll = imu.orientation.x

    actual.w_z = pitch_dot * (math.pi / 180)
    actual.roll = roll

    t.t_current = time()
    t.delta_t = t.t_current - t.t_prev
    # TODO: Try to write a better integration code
    actual.v_x += imu.linear_acceleration.x * t.delta_t
    actual.v_y += imu.linear_acceleration.y * t.delta_t
    t.t_prev = t.t_current
    rospy.loginfo(f"v_x: {round(actual.v_x)}, v_y: {round(actual.v_y)}")


def kinematic_control(pid_val_x: float, pid_val_y: float, pid_val_w: float) -> List:
    """_summary_

    Args:
        pid_val_x (float): PID value calculated for the error in x velocity
        pid_val_y (float): PID value calculated for the error in y velocity
        pid_val_w (float): PID value calculated for the error in w velocity

    Returns:
        List: List of four thrusters values responsible for the planner kinematic
        motion of the ROV
    """
    # Planer Control
    phi_2 = pid_val_x - pid_val_y
    phi_1 = pid_val_x + pid_val_y
    phi_3 = pid_val_x - pid_val_y
    phi_4 = pid_val_x + pid_val_y

    # Rotation Control
    phi_1 += pid_val_w
    phi_2 += -pid_val_w
    phi_3 += pid_val_w
    phi_4 += -pid_val_w

    return [phi_1, phi_2, phi_3, phi_4]


def find_phi_boudary_values_velocity_based(param: PARAM) -> List:
    """Estimate the thrusters boundary value solely based on max allowed velocity
        in each axis

    Args:
        param (PARAM): PARAM data class

    Returns:
        List: Boundary values for the thrusters actuation
    """
    max_phi_val = param.max_vx + param.max_vy + param.max_wz
    min_phi_val = param.min_vx + param.min_vy + param.min_wz
    return [min_phi_val, max_phi_val]


def find_phi_boudary_values_pid_based(param: PARAM) -> List:
    """Estimate the thrusters boundary value based on both the max allowed velocity
        in each axis and the PID parameters

    Args:
        param (PARAM): PARAM data class

    Returns:
        List: Boundary values for the thrusters actuation
    """
    abs_max_e_vx = abs(param.max_vx) + abs(param.min_vx)
    abs_max_de_vx = abs(param.max_vx)
    abs_max_e_sum_vx = (
        abs(param.max_vx) / 2
    )  # This is absolute and complete random estimation
    max_pid_val_vx = (
        abs_max_e_vx * param.kp_depth
        + abs_max_e_sum_vx * param.ki_depth
        + abs_max_de_vx * param.kd_depth
    )
    min_pid_val_vx = -max_pid_val_vx

    abs_max_e_vy = abs(param.max_vy) + abs(param.min_vy)
    abs_max_de_vy = abs(param.max_vy)
    abs_max_e_sum_vy = (
        abs(param.max_vy) / 2
    )  # This is absolute and complete random estimation

    max_pid_val_vy = (
        abs_max_e_vy * param.kp_depth
        + abs_max_e_sum_vy * param.ki_depth
        + abs_max_de_vy * param.kd_depth
    )
    min_pid_val_vy = -max_pid_val_vy

    abs_max_e_wz = abs(param.max_wz) + abs(param.min_wz)
    abs_max_de_wz = abs(param.max_wz)
    abs_max_e_sum_wz = (
        abs(param.max_wz) / 2
    )  # This is absolute and complete random estimation
    max_pid_val_wz = (
        abs_max_e_wz * param.kp_depth
        + abs_max_e_sum_wz * param.ki_depth
        + abs_max_de_wz * param.kd_depth
    )
    min_pid_val_wz = -max_pid_val_wz

    max_phi_val = max_pid_val_vx + max_pid_val_vy + max_pid_val_wz
    min_phi_val = min_pid_val_vx + min_pid_val_vy + min_pid_val_wz
    return [min_phi_val, max_phi_val]


def saturate_actuators(
    phi_1: float,
    phi_2: float,
    phi_3: float,
    phi_4: float,
    min_phi_val: float,
    max_phi_val: float,
) -> List:
    """_summary_

    Args:
        phi_1 (float): Unsaturated thruster 1 value in the ROV configuration
        phi_2 (float): Unsaturated thruster 2 value in the ROV configuration
        phi_3 (float): Unsaturated thruster 3 value in the ROV configuration
        phi_4 (float): Unsaturated thruster 4 value in the ROV configuration
        min_phi_val (float): Reverse saturation value
        max_phi_val (float): Forward saturation value

    Returns:
        List: Saturated list of four thrusters values responsible for the planner kinematic
        motion of the ROV
    """
    phi_1 = max(phi_1, min_phi_val)
    phi_1 = min(phi_1, max_phi_val)
    phi_2 = max(phi_2, min_phi_val)
    phi_2 = min(phi_2, max_phi_val)
    phi_3 = max(phi_3, min_phi_val)
    phi_3 = min(phi_3, max_phi_val)
    phi_4 = max(phi_4, min_phi_val)
    phi_4 = min(phi_4, max_phi_val)
    return [phi_1, phi_2, phi_3, phi_4]


def cmd_vel_recieved_callback(cmd_vel):
    """callback function for the subscriber to the ROV/cmd_vel topic

    Args:
        msg (Twist): command velocity sent by the control node
    """

    w_z = cmd_vel.angular.z * PARAM.kp_w
    v_x = cmd_vel.linear.x * PARAM.kp_x
    v_y = cmd_vel.linear.y * PARAM.kp_y
    d_z = cmd_vel.linear.z * PARAM.kp_depth

    # pid_val_x = pid_vx.compute(v_x, 0)
    # pid_val_y = pid_vy.compute(v_y, 0)
    # pid_val_w = pid_wz.compute(w_z, actual.w_z)

    # phi_1, phi_2, phi_3, phi_4 = kinematic_control(pid_val_x, pid_val_y, pid_val_w)
    phi_1, phi_2, phi_3, phi_4 = kinematic_control(v_x, v_y, w_z)
    # min_phi_val, max_phi_val = find_phi_boudary_values_pid_based(PARAM)
    min_phi_val, max_phi_val = find_phi_boudary_values_velocity_based(PARAM)

    phi_1, phi_2, phi_3, phi_4 = saturate_actuators(
        phi_1,
        phi_2,
        phi_3,
        phi_4,
        min_phi_val,
        max_phi_val,
    )

    phi_1 = map_from_to(
        phi_1,
        min_phi_val,
        max_phi_val,
        PARAM.thruster_min,
        PARAM.thruster_max,
    )
    phi_2 = map_from_to(
        phi_2,
        min_phi_val,
        max_phi_val,
        PARAM.thruster_min,
        PARAM.thruster_max,
    )
    phi_3 = map_from_to(
        phi_3,
        min_phi_val,
        max_phi_val,
        PARAM.thruster_min,
        PARAM.thruster_max,
    )
    phi_4 = map_from_to(
        phi_4,
        min_phi_val,
        max_phi_val,
        PARAM.thruster_min,
        PARAM.thruster_max,
    )

    planer_thrusters_list = [phi_2, phi_1, phi_3, phi_4]

    # # Depth Control and Stabilization
    # pid_depth_val = (
    #     pid_depth.compute(ref=d_z, measured=actual.depth) + PARAM.floatability
    # )
    # # pid_stabilization_val = pid_stabilization.compute(
    # #     ref=PARAM.roll_desired, measured=actual.roll
    # # )  # stabilization is done for the roll angle

    # abs_max_e_depth = PARAM.max_pool_depth
    # abs_max_de_depth = abs_max_e_depth / 2
    # abs_max_e_sum_depth = (
    #     abs_max_e_depth / 2
    # )  # This is absolute and complete random estimation
    # min_pid_val_depth = (
    #     -abs_max_e_depth * PARAM.kp_depth
    #     - abs_max_e_sum_depth * PARAM.ki_depth
    #     - abs_max_de_depth * PARAM.kd_depth
    # )
    # max_pid_val_depth = (
    #     abs_max_e_depth * PARAM.kp_depth
    #     + abs_max_e_sum_depth * PARAM.ki_depth
    #     + abs_max_de_depth * PARAM.kd_depth
    # )

    # # abs_max_e_roll = PARAM.max_roll
    # # abs_max_de_roll = abs_max_e_roll / 2
    # # abs_max_e_sum_roll = (
    # #     abs_max_e_roll / 2
    # # )  # This is absolute and complete random estimation
    # # min_pid_val_roll = (
    # #     -abs_max_e_roll * PARAM.kp_roll
    # #     - abs_max_e_sum_roll * PARAM.ki_roll
    # #     - abs_max_de_roll * PARAM.kd_roll
    # # )
    # # max_pid_val_roll = (
    # #     abs_max_e_roll * PARAM.kp_roll
    # #     + abs_max_e_sum_roll * PARAM.ki_roll
    # #     + abs_max_de_roll * PARAM.kd_roll
    # # )

    # depth_actuation = map_from_to(
    #     pid_depth_val,
    #     min_pid_val_depth,
    #     max_pid_val_depth,
    #     PARAM.thruster_min,
    #     PARAM.thruster_max,
    # )

    # # stabilization_actuation = map_from_to(
    # #     pid_stabilization_val,
    # #     min_pid_val_roll,
    # #     max_pid_val_roll,
    # #     PARAM.thruster_min,
    # #     PARAM.thruster_max,
    # # )
    depth_actuation = d_z*PARAM.kp_depth

    depth_actuation = map_from_to(
        depth_actuation,
        -1,
        1,
        1300,
        1650,
    )

    phi_5 = depth_actuation  # - stabilization_actuation
    phi_6 = depth_actuation  # + stabilization_actuation

    

    phi_5 = max(phi_5, 1300)
    phi_6 = max(phi_6, 1300)
    phi_5 = min(phi_5, 1650)
    phi_6 = min(phi_6, 1650)

    # phi_5 = PARAM.phi_rev
    # phi_6 = 1500

    planer_thrusters_list.append(phi_5)
    planer_thrusters_list.append(phi_6)

    # create the message instance
    thrusters_voltages = Float32MultiArray()

    # fill the message with the phi values
    thrusters_voltages.data = planer_thrusters_list

    # publish the message
    rospy.loginfo(thrusters_voltages)

    rospy.loginfo(
        f"kp_x: {round(PARAM.kp_x)}, ki_x: {round(PARAM.ki_x)}, kd_x: {round(PARAM.kd_x)}"
    )
    rospy.loginfo(
        f"kp_y: {round(PARAM.kp_y)}, ki_y: {round(PARAM.ki_y)}, kd_y: {round(PARAM.kd_y)}"
    )
    rospy.loginfo(
        f"kp_w: {round(PARAM.kp_w)}, ki_w: {round(PARAM.ki_w)}, kd_w: {round(PARAM.kd_w)}"
    )
    rospy.loginfo(
        f"kp_depth: {round(PARAM.kp_depth)}, ki_depth: {round(PARAM.ki_depth)}, kd_depth: {round(PARAM.kd_depth)}"
    )
    rospy.loginfo(
        f"kp_roll: {round(PARAM.kp_roll)}, ki_roll: {round(PARAM.ki_roll)}, kd_roll: {round(PARAM.kd_roll)}"
    )
    rospy.loginfo(f"floatability: {round(PARAM.floatability)}")
    rospy.loginfo(f"max_phi_val = {round(max_phi_val)}, min_phi_val = {round(min_phi_val)}")

    thrusters_voltages_publisher.publish(thrusters_voltages)
    rospy.sleep(0.01)

def stop_all():
    thrusters_voltages = Float32MultiArray()
    thrusters_voltages.data = [1500,1500,1500,1500,1500,1500]
    thrusters_voltages_publisher.publish(thrusters_voltages)


if __name__ == "__main__":
    t.t_prev = time()
    node_init()
