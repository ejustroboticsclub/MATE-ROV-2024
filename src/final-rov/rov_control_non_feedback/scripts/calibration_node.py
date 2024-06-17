"""Callibration of all the parameters that could be tuned
        the tuning could be sent via the joystick buttons or the ros topic pup
"""
import dataclasses
from time import time
import math
from typing import List
import rospy
from geometry_msgs.msg import Twist
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
    max_vx = 0.7
    min_vx = -0.7
    max_vy = 0.7
    min_vy = -0.7
    max_wz = 0.7
    min_wz = -0.7
    max_pool_depth = 2  # wil be 4 in the competition
    max_roll = 90
    min_roll = -90

    min_pid_depth = -15
    max_pid_depth = 15

    alpha = 0.7


@dataclasses.dataclass
class Measured:
    """Sensor measurments"""

    v_x = 0
    v_y = 0
    v_z = 0
    w_z = 0
    roll = 0
    depth_pressure = 0
    depth_pressure_prev = 0
    depth_imu = 0
    depth_kalman_filter = 0


@dataclasses.dataclass
class Time:
    """Time related variables for integration and differentiation"""

    t_prev = 0
    t_current = 0
    delta_t = 0


@dataclasses.dataclass
class KalmanFilter:
    """Kalman Filter related variables"""

    sigma_depth = 0.5
    sigma_imu = 0.2
    a = 1
    p_0 = 500
    p = 0
    q = sigma_depth**2
    r = sigma_imu**2
    g = 0


@dataclasses.dataclass
class ROSPublishers:
    """ROS Publishers"""

    thrusters_voltages_publisher = rospy.Publisher(
        "ROV/thrusters", Float32MultiArray, queue_size=10
    )


PARAM = Param()
actual = Measured()
t = Time()
kalman_filter = KalmanFilter()
ros_publishers = ROSPublishers()
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


def main():
    """Main Function"""
    # Prediction Step
    # x̂_k = A x̂_k−1 + B u kT
    # P_k = A P_k−1 A + Q

    # Update Step
    # G_k = P_k CT (C P_k CT + R)^−1
    # x̂_k ← x̂_k + G_k ∗ (z_k − C x̂_k )
    # P_k ← (I − G_k C )P_k

    # Prediction model is depth sensor
    # Sensor model is imu
    node_init()


def node_init():
    """Initialize motion_control node which subscribes to ROV/cmd_vel topic and the ROV/depth
    and publishes on the ROV/thrusters topic"""
    rospy.init_node("kinematic_model", anonymous=True)

    rospy.on_shutdown(stop_all)  # stop all thrusters

    # subscribers for communicating with other subsystems of the ROV to get the measurements of
    # depth, imu (linear and angular velocities), and desired velocities and depth from the joystick
    rospy.Subscriber("ROV/cmd_vel", Twist, cmd_vel_recieved_callback)
    rospy.Subscriber("ROV/depth", Float64, depth_recieved_callback)
    rospy.Subscriber("ROV/imu", Imu, imu_recieved_callback)

    srv = Server(pidConfig, set_param_callback)
    rospy.spin()


def set_param_callback(config, level):
    """Callback function for handling changes to ROS parameters.

    This function updates the values of various parameters used by the system based on the input
    configuration. If the input level is 0, the current parameter values are returned without
    modification.

    Args:
        config (dict): A dictionary containing the new parameter values.
        level (int): The level of the parameter update (0 = default, 1 = reload, 2 = dynamic).

    Returns:
        dict: The updated parameter configuration.
    """
    if not level:
        # Return the current parameter values without modification
        config["kp_depth"] = PARAM.kp_depth
        config["ki_depth"] = PARAM.ki_depth
        config["kd_depth"] = PARAM.kd_depth
        config["kp_x"] = PARAM.kp_x
        config["ki_x"] = PARAM.ki_x
        config["kd_x"] = PARAM.kd_x
        config["kp_y"] = PARAM.kp_y
        config["ki_y"] = PARAM.ki_y
        config["kd_y"] = PARAM.kd_y
        config["kp_w"] = PARAM.kp_w
        config["ki_w"] = PARAM.ki_w
        config["kd_w"] = PARAM.kd_w
        config["kp_roll"] = PARAM.kp_roll
        config["ki_roll"] = PARAM.ki_roll
        config["kd_roll"] = PARAM.kd_roll
        config["floatability"] = PARAM.floatability
        return config

    # Update the parameter values based on the input configuration
    PARAM.kp_depth = config["kp_depth"]
    PARAM.ki_depth = config["ki_depth"]
    PARAM.kd_depth = config["kd_depth"]
    PARAM.kp_x = config["kp_x"]
    PARAM.ki_x = config["ki_x"]
    PARAM.kd_x = config["kd_x"]
    PARAM.kp_y = config["kp_y"]
    PARAM.ki_y = config["ki_y"]
    PARAM.kd_y = config["kd_y"]
    PARAM.kp_w = config["kp_w"]
    PARAM.ki_w = config["ki_w"]
    PARAM.kd_w = config["kd_w"]
    PARAM.kp_roll = config["kp_roll"]
    PARAM.ki_roll = config["ki_roll"]
    PARAM.kd_roll = config["kd_roll"]
    PARAM.floatability = config["floatability"]

    return config


def depth_recieved_callback(depth):
    """callback function for the subscriber to the ROV/depth topic

    Args:
        msg (Float64): depth sent by the the depth sensor
    """
    actual.depth_pressure = depth.data
    actual.depth_pressure = low_pass_filter(
        actual.depth_pressure_prev, actual.depth_pressure
    )
    actual.depth_pressure_prev = actual.depth_pressure


def low_pass_filter(prev_value: float, new_value: float) -> float:
    """Low Pass Filter implementation

    Args:
        prev_value (float): Previous sensor reading
        new_value (float): Current sensor reading

    Returns:
        float: Filtered sensor reading
    """
    return PARAM.alpha * prev_value + (1 - PARAM.alpha) * new_value


def inegrate(x_dot: float, x_0: float) -> List:
    """Numerical integration funtion

    Args:
        dx (float): Variable to be integrated
        x_0 (float): Initial value

    Returns:
        List: integrated value, final time
    """
    t.t_current = time()
    t.delta_t = t.t_current - t.t_prev
    x_t = x_0 + x_dot * t.delta_t
    t.t_prev = t.t_current
    return [x_t, t.t_prev]


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

    # Integrate to get Velocity
    actual.v_x, t.t_prev = inegrate(imu.linear_acceleration.x, actual.v_x)
    actual.v_y, t.t_prev = inegrate(imu.linear_acceleration.z, actual.v_y)
    actual.v_z, t.t_prev = inegrate(imu.linear_acceleration.y, actual.v_z)

    actual.v_x = -actual.v_x
    actual.v_y = -actual.v_y
    actual.v_z = -actual.v_z

    # Integrate to get Position (depth)
    actual.depth_imu, t.t_prev = inegrate(actual.v_z, actual.depth_imu)

    x_sensor = actual.depth_imu
    # PREDICTION:-
    # prediction step
    x_predict = actual.depth_pressure
    # Prediciton uncertainty update
    kalman_filter.p = kalman_filter.a * kalman_filter.p_0 * kalman_filter.a
    # UPDATE:-
    # Kalman Gain update
    kalman_filter.g = kalman_filter.p / (kalman_filter.p + kalman_filter.r)
    # correction of predicted state
    x_predict = x_predict + kalman_filter.g * (x_sensor - x_predict)

    kalman_filter.p = (1 - kalman_filter.g) * kalman_filter.p
    kalman_filter.p_0 = kalman_filter.p

    actual.depth_kalman_filter = x_predict


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


def saturate_actuator(
    phi: float,
    min_phi_val: float,
    max_phi_val: float,
) -> int:
    """Saturate Thruster values

    Args:
        phi (float): Unsaturated thruster value
        min_phi_val (float): Reverse saturation value
        max_phi_val (float): Forward saturation value

    Returns:
        int: Saturated thruster value
    """
    phi_sat = phi
    phi_sat = max(phi_sat, min_phi_val)
    phi_sat = min(phi_sat, max_phi_val)
    return phi_sat


def saturate_actuator_list(
    acutuators_list: List,
    min_phi_val: float,
    max_phi_val: float,
) -> List:
    """_summary_

    Args:
        acutuators_list (List): List of actuators to be saturated
        min_phi_val (float): Reverse saturation value
        max_phi_val (float): Forward saturation value

    Returns:
        List: List of saturated actuation values
    """
    sat_actuators_list = []
    for phi in acutuators_list:
        phi_sat = saturate_actuator(
            phi,
            min_phi_val,
            max_phi_val,
        )
        sat_actuators_list.append(phi_sat)
    return sat_actuators_list


def cmd_vel_recieved_callback(cmd_vel):
    """callback function for the subscriber to the ROV/cmd_vel topic

    Args:
        msg (Twist): command velocity sent by the control node
    """

    w_z = cmd_vel.angular.z
    v_x = cmd_vel.linear.x
    v_y = cmd_vel.linear.y
    d_z = cmd_vel.linear.z

    pid_val_x = pid_vx.compute(v_x, actual.v_x)
    pid_val_y = pid_vy.compute(v_y, actual.v_y)
    pid_val_w = pid_wz.compute(w_z, actual.w_z)

    phi_1, phi_2, phi_3, phi_4 = kinematic_control(pid_val_x, pid_val_y, pid_val_w)
    # min_phi_val, max_phi_val = find_phi_boudary_values_pid_based(PARAM)
    min_phi_val, max_phi_val = find_phi_boudary_values_velocity_based(PARAM)

    actuators_list = [phi_1, phi_2, phi_3, phi_4]
    phi_1, phi_2, phi_3, phi_4 = saturate_actuator_list(
        actuators_list,
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

    planar_thrusters_list = [phi_2, phi_1, phi_3, phi_4]

    # Depth Control and Stabilization
    pid_depth_val = (
        pid_depth.compute(ref=d_z, measured=actual.depth_kalman_filter)
        + PARAM.floatability
    )
    pid_stabilization_val = pid_stabilization.compute(
        ref=PARAM.roll_desired, measured=actual.roll
    )  # stabilization is done for the roll angle

    phi_5 = pid_depth_val - pid_stabilization_val
    phi_6 = pid_depth_val + pid_stabilization_val

    depth_actuator_list = [phi_5, phi_6]

    phi_5, phi_6 = saturate_actuator_list(
        depth_actuator_list, PARAM.min_pid_depth, PARAM.max_pid_depth
    )

    phi_5 = map_from_to(
        phi_5,
        PARAM.min_pid_depth,
        PARAM.max_pid_depth,
        PARAM.thruster_min,
        PARAM.thruster_max,
    )

    phi_6 = map_from_to(
        phi_6,
        PARAM.min_pid_depth,
        PARAM.max_pid_depth,
        PARAM.thruster_min,
        PARAM.thruster_max,
    )

    # planar_thrusters_list.append(phi_5)
    # planar_thrusters_list.append(phi_6)

    planar_thrusters_list.append(1500)
    planar_thrusters_list.append(1500)

    # create the message instance
    thrusters_voltages = Float32MultiArray()

    # fill the message with the phi values
    thrusters_voltages.data = planar_thrusters_list

    # publish the message
    rospy.loginfo(thrusters_voltages)

    # rospy.loginfo(f"kp_x: {PARAM.kp_x}, ki_x: {PARAM.ki_x}, kd_x: {PARAM.kd_x}")
    # rospy.loginfo(f"kp_y: {PARAM.kp_y}, ki_y: {PARAM.ki_y}, kd_y: {PARAM.kd_y}")
    # rospy.loginfo(f"kp_w: {PARAM.kp_w}, ki_w: {PARAM.ki_w}, kd_w: {PARAM.kd_w}")
    # rospy.loginfo(
    #     f"kp_depth: {PARAM.kp_depth}, ki_depth: {PARAM.ki_depth}, kd_depth: {PARAM.kd_depth}"
    # )
    # rospy.loginfo(
    #     f"kp_roll: {PARAM.kp_roll}, ki_roll: {PARAM.ki_roll}, kd_roll: {PARAM.kd_roll}"
    # )
    # rospy.loginfo(f"max_phi_val = {max_phi_val}, min_phi_val = {min_phi_val}")

    rospy.loginfo(f"depth_pressure = {actual.depth_pressure}")
    rospy.loginfo(f"depth_imu = {actual.depth_imu}")
    rospy.loginfo(f"desired_depth = {d_z}")
    rospy.loginfo(f"depth = {actual.depth_kalman_filter}")

    ros_publishers.thrusters_voltages_publisher.publish(thrusters_voltages)
    rospy.sleep(0.01)


def stop_all():
    """Stops all thrusters by sending 1500 for all thrusters"""
    thrusters_voltages = Float32MultiArray()
    thrusters_voltages.data = [1500, 1500, 1500, 1500, 1500, 1500]
    ros_publishers.thrusters_voltages_publisher.publish(thrusters_voltages)


if __name__ == "__main__":
    t.t_prev = time()
    node_init()
