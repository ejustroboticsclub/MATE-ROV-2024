import dataclasses
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator


cam_1 = 0
cam_2 = 1

model = YOLO('C:/Users/MENNA/Documents/YOLO v8/runs/detect/train5/weights/best.pt')
cap = cv2.VideoCapture(cam_1)
cap.set(3, 440)
cap.set(4, 480)

cap_second_camera = cv2.VideoCapture(cam_2)  # Use camera device index or video file path for the second camera
cap_second_camera.set(3, 640)
cap_second_camera.set(4, 480)

def calculate_centroid_and_width(box):
    x1, y1, x2, y2 = box  # Unpack the box coordinates
    centroid_x = int((x1 + x2) / 2)
    centroid_y = int((y1 + y2) / 2)
    width = int(abs(x1 - x2))
    return centroid_x, centroid_y, width

def calculate_area(box):
    x1, y1, x2, y2 = box  
    return abs((x1-x2) * (y1-y2))

def first_camera_detection():
    red_square_detected = False
    if cap.isOpened():
        _, frame = cap.read()

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = model.predict(img)

        for r in results:
            annotator = Annotator(frame)

            boxes = r.boxes
            for box in boxes:
                if box.conf > 0.8:
                    red_square_detected = True
                    b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])

                    # Calculate centroid and width
                    centroid_x, centroid_y, width = calculate_centroid_and_width(b)
                    print(f"Centroid of bounding box and width: ({centroid_x}, {centroid_y}, {width})")
                    
                    # Calculate area
                    area = calculate_area(b)
                    print(f"Area of bounding box: ({area})")

                    # Draw centroid on frame
                    cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
                
                else: 
                    red_square_detected = False

    frame = annotator.result()
    cv2.imshow('YOLO V8 Detection', frame)

    cap.release()
    cv2.destroyAllWindows()

    return red_square_detected, centroid_x, centroid_y, width, area


def second_camera_detection():

    def calculate_bounding_vertex(box):
        x1, y1, x2, y2 = box  
        return x1, y1, abs(x2-x1)


    if cap_second_camera.isOpened():
        _, frame_second_camera = cap_second_camera.read()
        
        # Assume the model predicts a part of the frame (adjusted during training)
        height, width, _ = frame_second_camera.shape
        
        img = cv2.cvtColor(frame_second_camera, cv2.COLOR_BGR2RGB)

        results = model.predict(img)

        red_square_detected_upper_half = False

        def calculate_blind_vertex():
            return width//4, height//2
        blind_x, blind_y = calculate_blind_vertex()

        for r in results:
            annotator = Annotator(frame_second_camera)
            boxes = r.boxes
            for box in boxes:
                if(box.conf > 0.7):
                    b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                    c = box.cls
                    if (b[0] < width//4 or b[2] > 3*(width//4)) or b[1] < height//2:
                        annotator.box_label(b, model.names[int(c)])
                        red_square_detected_upper_half = True
                        x1, y1, bounding_width = calculate_bounding_vertex(b)
                        print(f"Bounding vertex: ({x1}, {y1}, {bounding_width})")
                    
                    else: 
                        red_square_detected_upper_half = False
            
        '''
        if red_square_detected_upper_half:
            # Adjust the frame for releasing the brain coral on the red square
            adjust_frame_for_release(x=desired_x_coordinate, y=desired_y_coordinate)
        '''
        frame_second_camera = annotator.result() 
        cv2.imshow('Second Camera - YOLO V8 Detection', frame_second_camera)

    cap_second_camera.release()
    cv2.destroyAllWindows()

    return red_square_detected_upper_half,  x1, y1, bounding_width,  blind_x, blind_y