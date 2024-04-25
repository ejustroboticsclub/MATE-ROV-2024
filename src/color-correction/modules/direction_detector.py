"""
   Name : Abdelrahman Amr Abdelrahman El Sayed 
"""
import cv2
import numpy as np

class RedStickDetector:
    def __init__(self, image_path):
        self.image_path = image_path
        self.image = cv2.imread(image_path)

    def detect_red_stick(self):
        # Take the lower 80% of the image 
        # (because the x button in the image viewer bar was red-colored 
        # so it interefered with the algorithm XD)
        
        height, _, _ = self.image.shape
        roi = self.image[int(height*0.2):height, :]  

        # Thresholding to isolate red color
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([100, 100, 255])
        mask = cv2.inRange(roi, lower_red, upper_red)

        edges = cv2.Canny(mask, 100, 200)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calculate center of the red stick and the frame
        if len(contours) > 0:
            red_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(red_contour)
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            red_center = (center_x, center_y)

            frame_center = (roi.shape[1] // 2, roi.shape[0] // 2)

            # draw a circle at the center of the red stick
            cv2.circle(roi, red_center, 5, (0, 255, 0), -1)

            # draw a circle at the center of the frame
            cv2.circle(roi, frame_center, 5, (255, 0, 0), -1)

            # draw a line from the center of the frame to the red stick
            cv2.line(roi, frame_center, red_center, (0, 0, 255), 2)

            while True:
                cv2.imshow("Direction Visualization", roi)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # to handle interrupt keyboard 
                    #(don't know but control+C was not working with me XD)
                    cv2.destroyAllWindows()
                    break

            # Calculate direction vector from center of frame to red stick
            direction_vector = (center_x - frame_center[0], center_y - frame_center[1])

            return direction_vector
        else:
            return None

if __name__ == "__main__":
    print("Enter the number of the image you want to process:")
    image_number = input()
    image_path = f"/Users/python/Desktop/abdelrahman_python_task_6_robotics/color_corrected_images/{image_number}.jpeg"
    detector = RedStickDetector(image_path)
    direction_vector = detector.detect_red_stick()
    if direction_vector is not None:
        print(f"The next point to navigate towards is: {direction_vector}")
    else:
        print("No contours found.")
