#!/usr/bin/env python3
"""This is a library NOT a script. DON'T RUN THIS FILE OR USE IT'S MAIN FUNCTION.
The function to use in this module is called 'get_thickness_and_direction'.
"""
import rospy
import numpy as np
import cv2
from color_correct import correct
from enum import Enum
from typing import Optional, Tuple
from std_msgs.msg import Int16MultiArray

GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
RADIUS = 3
LINE_THICKNESS = 4
FONT = cv2.FONT_HERSHEY_COMPLEX
FONT_SCALE = 1
# tunable parameters
BLUR_KERNEL = (15, 15)
LOWER_BOUND_RED_1 = np.array([0, 75, 20])
UPPER_BOUND_RED_1 = np.array([10, 255, 255])
LOWER_BOUND_RED_2 = np.array([160, 75, 20])
UPPER_BOUND_RED_2 = np.array([180, 255, 255])
LINE_THRESHOLD = 90


class Direction(Enum):
    STOP = 0
    LEFT = 1
    RIGHT = 2
    UP = 3
    DOWN = 4


CUR_DIR = Direction.STOP
STOP = False


def correct_color_underwater(img: np.ndarray) -> np.ndarray:
    """This function shifts the color of the input image to correct the color shift underwater.

    Args:
        img (np.ndarray): input image is in BGR format of any size.

    Returns:
        np.ndarray: color-corrected image of the underwater image.
    """
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    corrected_img = correct(img_rgb)
    return corrected_img


def apply_filter(img: np.ndarray) -> np.ndarray:
    """Applies Preprocessing filters such as gaussian blur.

    Args:
        img(np.ndarray): input image to be preprocessed.

    Returns:
        np.ndarray: processed image.
    """
    img = correct_color_underwater(img)
    img = cv2.GaussianBlur(img, BLUR_KERNEL, cv2.BORDER_DEFAULT)
    return img


def get_red_mask(img: np.ndarray) -> np.ndarray:
    """Converts image to HSV to extract red color and returns red mask"""
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_mask: np.ndarray = cv2.inRange(img_hsv, LOWER_BOUND_RED_1, UPPER_BOUND_RED_1)
    upper_mask: np.ndarray = cv2.inRange(img_hsv, LOWER_BOUND_RED_2, UPPER_BOUND_RED_2)
    mask = lower_mask + upper_mask
    return mask


def get_lines(mask: np.ndarray) -> np.ndarray:
    """Finds lines in the image mask."""
    canny_image = cv2.Canny(
        mask, threshold1=50, threshold2=200, edges=None, apertureSize=3
    )
    lines = cv2.HoughLines(canny_image, 1, np.pi / 180, LINE_THRESHOLD, None, 0, 0)
    return lines


def get_center_moment(mask: np.ndarray) -> Tuple[int, int]:
    """Takes as input binary image and returns the center of white pixels.

    Args:
        mask(np.ndarray): Input binary image.

    Returns:
        Tuple[int,int]: Cx, and Cy of the image.
    """
    moments = cv2.moments(mask)
    if np.isclose(moments["m00"], 0.0):
        return 0, 0

    cX = int(moments["m10"] / moments["m00"])
    cY = int(moments["m01"] / moments["m00"])
    return cX, cY


def get_direction(mask: np.ndarray) -> Direction:
    """Takes as input binary image and returns the direction of the line.

    Args:
        mask(np.ndarray): Input binary image.

    Returns:
        (Direction): Direction of the line."""
    global CUR_DIR
    global STOP
    if STOP:
        print("task is done")
        return Direction.STOP
    line_up = np.any(mask[0, :])
    line_down = np.any(mask[-1, :])
    line_left = np.any(mask[:, 0])
    line_right = np.any(mask[:, -1])
    no_of_edges = sum(map(int, [line_up, line_down, line_left, line_right]))
    # if more than 2 edges are detected consider it as an error and keep current state.
    if no_of_edges > 2:
        return CUR_DIR
    # if only one edge is detected then it is either the start or the end of the track.
    if no_of_edges == 1:
        if line_up and CUR_DIR == Direction.DOWN:
            STOP = True
            CUR_DIR = Direction.STOP
            return CUR_DIR
        if line_up:
            CUR_DIR = Direction.UP
            return CUR_DIR
    # always prioritize right over all other directions.
    if line_right:
        CUR_DIR = Direction.RIGHT
        return CUR_DIR

    if line_left:
        if line_up:
            CUR_DIR = Direction.UP
            return CUR_DIR
        if line_down:
            CUR_DIR = Direction.DOWN
            return CUR_DIR
    # if no right or left edges are detected then it is a straight line.
    if line_up and line_down:
        return CUR_DIR

    print("this current state is not handled, returning STOP, please check the code.")
    return Direction.STOP


def get_thickness_and_direction(
    img: np.ndarray,
) -> Tuple[int, Tuple[int, int], Tuple[int, int], Tuple[int, int]]:
    """applies preprocessing, red line detection, and finds thickness for line if exists.
    Returns None if no lines exist.

    Args:
        img(np.ndarray): Input image frame from video feed.

    Returns:
        thickness(int): Thickness of the red line in pixels.
        next_point(Tuple[int,int]): the x,y coordinate of the line in the upper half of frame.
        middle_point(Tuple[int,int]): the x,y coordinate of the line in the whole frame.
        prev_point(Tuple[int,int]): the x,y coordinate of the line in the lower half of the frame .
    """
    # TODO: agree on a specefic output format if there are no detected lines in the image for thickness and direction.(maybe use get_lines func)

    img = apply_filter(img)
    mask = get_red_mask(img)
    direction = get_direction(mask)
    # kernel = np.ones((5, 5), np.uint8)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    ##### debug
    cv2.putText(mask, f"direction = {direction}", (50, 50), FONT, FONT_SCALE, GREEN, 5)
    cv2.imshow("mask", mask)
    cv2.waitKey(0)
    ####
    if direction == Direction.STOP:
        next_point = img.shape[1] // 2, img.shape[0] // 2
    elif direction == Direction.UP:
        next_point = img.shape[1] // 2, img.shape[0] // 4
    elif direction == Direction.DOWN:
        next_point = img.shape[1] // 2, img.shape[0] * 3 // 4
    else:
        next_point = img.shape[1] * 3 // 4, img.shape[0] // 2

    middle_point = img.shape[1] // 2, img.shape[0] // 2
    # calculate thickness of horizontal and vertical lines in mask by two scans.
    all_horizontal_thicknesses = np.count_nonzero(mask, axis=1)
    all_vertical_thicknesses = np.count_nonzero(mask, axis=0)
    # remove the zero values from both arrays
    all_horizontal_thicknesses = all_horizontal_thicknesses[
        all_horizontal_thicknesses != 0
    ]
    all_vertical_thicknesses = all_vertical_thicknesses[all_vertical_thicknesses != 0]
    # get the median of combined arrays
    thickness = int(
        np.median(
            np.concatenate((all_horizontal_thicknesses, all_vertical_thicknesses))
        )
    )
    return (
        thickness,
        next_point,
        middle_point,
        (0, 0),
    )

def node_init() -> None:
    """Initialize the line following node which  publishes on the ROV/line topic """

    rospy.init_node("line_task", anonymous=True)

    line_pub = rospy.Publisher("ROV/line",Int16MultiArray,queue_size=1)

    rate = rospy.Rate(10)
    
    line_msg = Int16MultiArray()

    IP = "rtsp://192.168.1.100:8554/unicast"
    pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink"
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    while 1: 
        
        _,img = cap.read()        

        thickness, (nextpt), (middle_pt), (prev_pt) = get_thickness_and_direction(img)
        
        line_msg.data = [thickness, nextpt[0], nextpt[1]]
        line_pub.publish(line_msg)

        cv2.circle(img, prev_pt, RADIUS, GREEN, 5)
        cv2.circle(img, middle_pt, RADIUS, GREEN, 5)
        cv2.circle(img, nextpt, RADIUS, GREEN, 5)
        # draw line with length = thickness to view the thickness visually
        cv2.line(img, (200, 200), (200 + thickness, 200), GREEN, LINE_THICKNESS)
        cv2.arrowedLine(img, middle_pt, nextpt, BLUE, LINE_THICKNESS)
        cv2.putText(
            img,
            f"thick = {thickness}",
            (50, 20),
            FONT,
            FONT_SCALE,
            GREEN,
            5,
        )
        cv2.imshow("Points", img)
        
        if(cv2.waitKey(1) == ord("q")):
            break
    
    cap.release
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    node_init()
