
from Automatic_control import AutomaticController
import dataclasses
from Detection import first_camera_detection, second_camera_detection

@dataclasses.dataclass
class Point:
    x = 0
    y = 0

@dataclasses.dataclass
class SquareParameters:
    length = 0
    desired_length = 0
    width = 0
    dims_ratio = length/width
    aligned = False

centroid_1 = Point()
square_upper_left_corner = Point()
gripper_upper_left_corner = Point()
square_1 = SquareParameters()
square_2 = SquareParameters()

#TODO: determine the value of the y axis
y_axis_center = 480/2 
x_axis_center = 640/2   # this point set is below the frame center
center_tolerance = 5
square_1.desired_length = 160


controller = AutomaticController()

depth = controller.depth
initial_orientation = controller.orientation
current_orientation = initial_orientation


# Mission is interrupted once the gripper release the coral head or manually by the pilot 
mission_interrupted = False

controller.ascent()

while not mission_interrupted: 


    first_square_detected,  centroid_x, centroid_y, width, area = first_camera_detection()

    centroid_1.x = centroid_x
    centroid_1.y = centroid_y
    square_1.width = width
    square_1.length = area/square_1.width

    second_square_detected,  x1, y1, bounding_width,  blind_x, blind_y = second_camera_detection()

    square_upper_left_corner.x = x1
    square_upper_left_corner.y = y1
    gripper_upper_left_corner.x = blind_x
    gripper_upper_left_corner.y = blind_y
    square_2.width = bounding_width
    square_2.length = area/square_2.width    


    if first_square_detected:
        # Perform ascent until 1 meter depth is reached 
        
        controller.node.get_logger().info('Square detected in frame 1', once=True)

        if centroid_1.x - y_axis_center > center_tolerance:
            square_1.aligned = False
            controller.right()
            

        elif centroid_1.x - y_axis_center > center_tolerance:
            square_1.aligned = False
            controller.left()

        # TODO: adjust the ratio threshold, 1 is ideal
        if (square_1.length < square_1.desired_length) \
            and (second_square_detected == False) \
            and (square_1.dims_ratio < 1):

            square_1.aligned = False
            if centroid_1.y - x_axis_center > center_tolerance:
                controller.forward_movement() 

            elif centroid_1.y - x_axis_center < -center_tolerance:
                controller.backward_movement() 
        else: 
            square_1.aligned = True

        if square_1.aligned:
            controller.node.get_logger().info('Square is aligned successfully', once=True)
        else:
            controller.node.get_logger().info('Aligning...', once=True)
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
        controller.node.get_logger().info('Scanning for square....', once=True)
        # Sweep in the right then left direction with a max deviation of 70 degrees until the red square is detected 
        if current_orientation - initial_orientation < 70:
            controller.rotate_right()
            first_square_detected = first_camera_detection()
            
        elif current_orientation - initial_orientation > -70:
            controller.rotate_left()
        
        current_orientation = controller.orientation
        
    
    controller.run_controller()

