import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64
from tf.transformations import euler_from_quaternion


class AutomaticController:

    def __init__(self):
        self.node = rclpy.create_node('Automatic_control')
        self.vel_publisher = self.node.create_publisher(Twist, 'ROV/cmd_vel', 10)
        self.depth_subscriber = self.node.create_subscription(Float64, 'ROV/depth', self.depth_callback, 10)
        self.imu = self.node.create_subscription(Imu, 'ROV/imu', self.imu_callback, 10)
        self.gripper_publisher = self.node.create_publisher(Bool, 'ROV/gripper', 1)

        self.velocity = Twist()
        self.gripper = Bool()

    def depth_callback(self, msg):
        self.depth = msg.data
        self.node.get_logger().info('Received depth: %f' % self.depth)
    
    def imu_callback(self, msg):
        _roll, _pith, yaw = euler_from_quaternion(
            [self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w]
        )
        self.orientation = yaw
        self.node.get_logger().info('Received orientation around yaw: %f' % self.orientation)

    def ascent(self):
        """Move the ROV upward until it's above the pool floor by 1 meter 
            according to the current depth reading.
            
        """
        #TODO: check that 1 meter is enough
        while self.depth < 1.0:
            self.velocity.linear.z = 1.0
            self.run_controller()
    

    def right(self):
        self.velocity.linear.y = 1.0 
    
    def left(self):
        self.velocity.linear.y = -1.0 

    def rotate_right(self):
        self.velocity.angular.z = 1.0 
    
    def rotate_left(self):
        self.velocity.angular.z = -1.0 

    '''
    def square_alignment(self, centroid):
        """Align the square's centroid with the y-axis.

        Args:
            centroid (float): Y-coordinate of the square's centroid.

        Returns:
            aligned (bool): Determine whether the square is aligned or not yet. 
        """

        #TODO: determine the value of the y axis
        y_axis_center = 0
        center_tolerance = 0.1
        if centroid < -center_tolerance + y_axis_center:
            self.velocity.linear.y = -1.0
            return False
        elif centroid > center_tolerance + y_axis_center:
            self.velocity.linear.y = 1.0
            return False
        else: 
            self.velocity.linear.y = 0.0
            return True
    '''

    def forward_movement(self):
        self.velocity.linear.x = 1.0
    
    def backward_movement(self):
        self.velocity.linear.x = -1.0

    def depth_adjustment_downward(self):
        self.velocity.linear.z = -1.0

    def depth_adjustment_upward(self):
        self.velocity.linear.z = -1.0

    def release_gripper(self):
        self.gripper.data = True

    def run_controller(self):
        timer = self.node.create_timer(0.1, timer_callback)

        def timer_callback():
            self.gripper_publisher.publish(self.gripper)
            self.vel_publisher.publish(self.velocity)

        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_timer(timer)
        self.node.destroy_node()
        rclpy.shutdown()
