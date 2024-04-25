"""
   Name : Abdelrahman Amr Abdelrahman El Sayed 
"""
import cv2
import numpy as np
import matplotlib.pyplot as plt

# helping module to visualize the green , red , blue channel (helped me to adjust parameters)


def histogram_analysis(image_path, threshold_ratio=0.01):
    image = cv2.imread(image_path)

    hist_r = cv2.calcHist([image], [0], None, [256], [0, 256])
    hist_g = cv2.calcHist([image], [1], None, [256], [0, 256])
    hist_b = cv2.calcHist([image], [2], None, [256], [0, 256])

    # Normalize histograms to 0-255 range
    cv2.normalize(hist_r, hist_r, 0, 255, cv2.NORM_MINMAX)
    cv2.normalize(hist_g, hist_g, 0, 255, cv2.NORM_MINMAX)
    cv2.normalize(hist_b, hist_b, 0, 255, cv2.NORM_MINMAX)

    threshold_level = (image.shape[0] * image.shape[1]) * threshold_ratio

    # Identify low-frequency colors based on the threshold level
    low_freq_r = np.where(hist_r < threshold_level)[0]
    low_freq_g = np.where(hist_g < threshold_level)[0]
    low_freq_b = np.where(hist_b < threshold_level)[0]

    # Plot histograms
    plt.figure(figsize=(10, 5))
    plt.title('Histogram Analysis')
    plt.plot(hist_r, color='r', label='Red')
    plt.plot(hist_g, color='g', label='Green')
    plt.plot(hist_b, color='b', label='Blue')
    plt.xlabel('Pixel Value')
    plt.ylabel('Frequency')
    plt.legend()
    plt.show()

    return {
        'low_freq_r': low_freq_r,
        'low_freq_g': low_freq_g,
        'low_freq_b': low_freq_b
    }


print("Enter the number of image to perform the analysis:")
image_number = input("Enter the number of the image: ")
result = histogram_analysis(f"/Users/python/Desktop/abdelrahman_python_task_6_robotics/color_corrected_images/{image_number}.jpeg", threshold_ratio=0.01)
print('Low-frequency red colors:', result['low_freq_r'])
print('Low-frequency green colors:', result['low_freq_g'])
print('Low-frequency blue colors:', result['low_freq_b'])
