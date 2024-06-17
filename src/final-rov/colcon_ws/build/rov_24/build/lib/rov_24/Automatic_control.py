import rclpy
from rclpy.node import Node
import dataclasses
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64
from tf_transformations import euler_from_quaternion
from .Detection import first_camera_detection, second_camera_detection

@dataclasses.dataclass
class Point:
    x = 0
    y = 0

@dataclasses.dataclass
class SquareParameters:
    length = 0
    desired_length = 0
    width = 0
    dims_ratio = 0
    aligned = False

centroid_1 = Point()
square_upper_left_corner = Point()
gripper_upper_left_corner = Point()
square_1 = SquareParameters()
square_2 = SquareParameters()

y_axis_center = 480/2 
x_axis_center = 640/2   # this point set is below the frame center
center_tolerance = 5
square_1.desired_length = 160


class AutomaticController(Node):

    def __init__(self):
        super().__init__('automatic_control')
        self.vel_publisher = self.create_publisher(Twist, 'ROV/cmd_vel', 10)
        self.depth_subscriber = self.create_subscription(Float64, 'ROV/depth', self.depth_callback, 10)
        self.imu = self.create_subscription(Imu, 'ROV/imu', self.imu_callback, 10)
        self.gripper_publisher = self.create_publisher(Bool, 'ROV/gripper', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)


        self.velocity = Twist()
        self.gripper = Bool()
        self.gripper.data = False
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0
        self.orientation = 0
        self.depth = 0
        
    def depth_callback(self, msg):
        self.depth = msg.data
        # self.get_logger().info('Received depth: %f' % self.depth)
    
    def imu_callback(self, msg):
        _roll, _pith, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        self.orientation = yaw
        # self.node.get_logger().info('Received orientation around yaw: %f' % self.orientation)

    def ascent(self):
        """Move the ROV upward until it's above the pool floor by 1 meter 
            according to the current depth reading.
            
        """
        #TODO: check that 1 meter is enough
        if self.depth < 60.0:
            self.velocity.linear.z = -1.0
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = 0.0
            self.velocity.angular.x = 0.0
            self.velocity.angular.y = 0.0
            self.velocity.angular.z = 0.0    

    def right(self):
        self.velocity.linear.y = 1.0 
        self.velocity.linear.x = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0
    
    def left(self):
        self.velocity.linear.y = -1.0
        self.velocity.linear.x = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

    def rotate_right(self):
        self.velocity.angular.z = 1.0
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        
    
    def rotate_left(self):
        self.velocity.angular.z = -1.0
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0

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
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0
    
    def backward_movement(self):
        self.velocity.linear.x = -1.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

    def depth_adjustment_downward(self):
        self.velocity.linear.z = 1.0
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

    def depth_adjustment_upward(self):
        self.velocity.linear.z = -1.0
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

    def release_gripper(self):
        self.gripper.data = True

    def timer_callback(self):
        initial_orientation = self.orientation
        current_orientation = initial_orientation


        # Mission is interrupted once the gripper release the coral head or manually by the pilot 
        mission_interrupted = False

        # self.ascent()

        if not mission_interrupted: 
            self.get_logger().info("Reached////////////////////////////////////////////////////////////////////////")
            first_square_detected,  centroid_x, centroid_y, width, area = first_camera_detection()

            centroid_1.x = centroid_x
            centroid_1.y = centroid_y
            square_1.width = width
            square_1.length = area/square_1.width
            square_1.dims_ratio = square_1.width / square_1.length

            second_square_detected,  x1, y1, bounding_width,  blind_x, blind_y = second_camera_detection()

            square_upper_left_corner.x = x1
            square_upper_left_corner.y = y1
            gripper_upper_left_corner.x = blind_x
            gripper_upper_left_corner.y = blind_y
            square_2.width = bounding_width
            square_2.length = area/square_2.width  

            right_search_done = False


            if first_square_detected:
                # Perform ascent until 1 meter depth is reached 
                
                self.get_logger().info('Square detected in frame 1', once=True)

                if centroid_1.x - y_axis_center > center_tolerance:
                    square_1.aligned = False
                    self.right()
                    

                elif centroid_1.x - y_axis_center > center_tolerance:
                    square_1.aligned = False
                    self.left()

                # TODO: adjust the ratio threshold, 1 is ideal
                if (square_1.length < square_1.desired_length) \
                    and (second_square_detected == False) \
                    and (square_1.dims_ratio < 0.7):

                    square_1.aligned = False
                    if centroid_1.y - x_axis_center > center_tolerance:
                        self.forward_movement() 

                    elif centroid_1.y - x_axis_center < -center_tolerance:
                        self.backward_movement() 
                else: 
                    square_1.aligned = True

                if square_1.aligned:
                    self.get_logger().info('Square is aligned successfully', once=True)
                else:
                    self.get_logger().info('Aligning...', once=True)
                '''
                # Approach the red square to get the gripper ready for releasing
                if second_square_detected == True and square_1.aligned:
                    if square_2.length < square_2.desired_length:
                        controller.depth_adjustment_downward()
                    else: 
                        controller.depth_adjustment_upward()

                    if square_upper_left_corner.x - gripper_upper_left_corner.x > center_tolerance:
                        controller.right()
                    elif square_upper_left_corner.x - gripper_upper_left_corner.x < -center_tolerance: 
                        controller.left()
                    
                    if square_upper_left_corner.y - gripper_upper_left_corner.y > center_tolerance:
                        controller.forward_movement()
                    elif square_upper_left_corner.y - gripper_upper_left_corner.y < -center_tolerance: 
                        controller.backward_movement()

                    else:
                        square_2.aligned = True

                if square_2.aligned:
                    controller.release_gripper()
                    mission_interrupted = True
                '''    

            else:
                self.get_logger().info('Searching for square....', once=True)
            #     # Sweep in the right then left direction with a max deviation of 70 degrees until the red square is detected 
            #     if (current_orientation - initial_orientation < 70) and (not right_search_done):
            #         self.rotate_right()
            #     else:
            #         right_search_done = True
            #         if not (current_orientation == initial_orientation):
            #             self.rotate_left()
                    
            #         if (current_orientation - initial_orientation > -70):
            #             self.rotate_left()
            #         else:
            #             right_search_done = False
            #             if not (current_orientation == initial_orientation):
            #                 self.rotate_right()
                
            #     current_orientation = self.orientation
        

        self.gripper_publisher.publish(self.gripper)
        self.vel_publisher.publish(self.velocity)



def main(args=None):
    rclpy.init(args=args)

    controller = AutomaticController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        # controller.get_logger().info('Shutting down...')
        controller.destroy_timer(controller.timer)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()