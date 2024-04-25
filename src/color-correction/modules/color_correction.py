"""
   Name : Abdelrahman Amr Abdelrahman El Sayed 
"""
import numpy as np
from PIL import Image

class ImageProcessor:
    def __init__(self, image_path):
        self.image = Image.open(image_path)
        self.image_array = np.array(self.image, dtype=np.float64)
        self.width, self.height, _ = self.image_array.shape

    def compensate_RB(self, flag):
        imageR, imageG, imageB = self.image.split()
        imageR = np.array(imageR, dtype=np.float64)
        imageG = np.array(imageG, dtype=np.float64)
        imageB = np.array(imageB, dtype=np.float64)

        imageR = (imageR - np.min(imageR)) / (np.max(imageR) - np.min(imageR))
        imageG = (imageG - np.min(imageG)) / (np.max(imageG) - np.min(imageG))
        imageB = (imageB - np.min(imageB)) / (np.max(imageB) - np.min(imageB))

        meanR = np.mean(imageR)
        meanG = np.mean(imageG)
        meanB = np.mean(imageB)

        if flag == 0:
            imageR = (imageR + (meanG - meanR) * (1 - imageR) * imageG) * 255
            imageB = (imageB + (meanG - meanB) * (1 - imageB) * imageG) * 255
        elif flag == 1:
            imageR = (imageR + (meanG - meanR) * (1 - imageR) * imageG) * 255

        imageR = np.clip(imageR, 0, 255).astype(np.uint8)
        imageG = np.clip(imageG * 255, 0, 255).astype(np.uint8)
        imageB = np.clip(imageB * 255, 0, 255).astype(np.uint8)

        compensated_im = Image.merge('RGB', (Image.fromarray(imageR), Image.fromarray(imageG), Image.fromarray(imageB)))

        return compensated_im


    def gray_world(self, output_path):
        imager, imageg, imageb = self.image.split()
        
        imagegray = self.image.convert('L')

        imageR = np.array(imager, dtype=np.float64)
        imageG = np.array(imageg, dtype=np.float64)
        imageB = np.array(imageb, dtype=np.float64)
        imageGray = np.array(imagegray, dtype=np.float64)
        
        x, y = self.width, self.height

        # Get mean value of pixels
        meanR = np.mean(imageR)
        meanG = np.mean(imageG)
        meanB = np.mean(imageB)
        meanGray = np.mean(imageGray)

        # Calculate scaling factors for each channel
        scaleR = meanGray / meanR
        scaleG = meanGray / meanG
        scaleB = meanGray / meanB

        # Apply scaling to each channel
        imageR = np.clip(imageR * scaleR, 0, 255).astype(np.uint8)
        imageG = np.clip(imageG * scaleG, 0, 255).astype(np.uint8)
        imageB = np.clip(imageB * scaleB, 0, 255).astype(np.uint8)

        # Create the white-balanced image
        whitebalanced_im = Image.merge('RGB', (Image.fromarray(imageR), Image.fromarray(imageG), Image.fromarray(imageB)))

        whitebalanced_im.save(output_path)

        return whitebalanced_im

    def show_original(self):
        self.image.show()

    def show_compensated(self, flag):
        compensated_im = self.compensate_RB(flag)
        compensated_im.show()

if __name__ == '__main__':
    print("Please enter the number of the image you want to process:")
    image_number = input()
    image_processor = ImageProcessor(f'/Users/python/Desktop/abdelrahman_python_task_6_robotics/images/{image_number}.jpeg')
    image_processor.gray_world(f"/Users/python/Desktop/abdelrahman_python_task_6_robotics/color_corrected_images/{image_number}.jpeg")
