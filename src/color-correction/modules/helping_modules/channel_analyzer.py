"""
   Name : Abdelrahman Amr Abdelrahman El Sayed 
"""
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np

# helping module to visualize the green , red , blue channel (helped me to adjust parameters)
class ChannelAnalyzer:
    def __init__(self, image_path):
        self.image = Image.open(image_path)
        self.width, self.height = self.image.size

    def channel_analysis(self):
        imager, imageg, imageb = self.image.split()
        
        imageR = np.array(imager, dtype=np.float64)
        imageG = np.array(imageg, dtype=np.float64)
        imageB = np.array(imageb, dtype=np.float64)

        # Plot histograms for each channel
        fig, axs = plt.subplots(1, 3, figsize=(15, 5))
        axs[0].hist(imageR.flatten(), bins=256, range=(0, 255), color='r', alpha=0.7)
        axs[0].set_title('Red Channel')
        axs[0].set_xlabel('Pixel Value')
        axs[0].set_ylabel('Frequency')

        axs[1].hist(imageG.flatten(), bins=256, range=(0, 255), color='g', alpha=0.7)
        axs[1].set_title('Green Channel')
        axs[1].set_xlabel('Pixel Value')
        axs[1].set_ylabel('Frequency')

        axs[2].hist(imageB.flatten(), bins=256, range=(0, 255), color='b', alpha=0.7)
        axs[2].set_title('Blue Channel')
        axs[2].set_xlabel('Pixel Value')
        axs[2].set_ylabel('Frequency')

        plt.tight_layout()
        plt.show()
if __name__ == '__main__':
    print("Please enter the number of the image you want to process:")
    image_number = input()
    channelAnalyzer = ChannelAnalyzer(f'/Users/python/Desktop/abdelrahman_python_task_6_robotics/images/{image_number}.jpeg')
    channelAnalyzer.channel_analysis()