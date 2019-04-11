
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Apply x or y gradient with the OpenCV Sobel() function
# and take the absolute value
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
# Rescale back to 8 bit integer
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
# Create a copy and apply the threshold
    grad_binary = np.zeros_like(scaled_sobel)
# Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
    grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

# Return the result
    return grad_binary


def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# Take both Sobel x and y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
# Calculate the gradient magnitude
    gradmag = np.sqrt(sobelx**2 + sobely**2)
# Rescale to 8 bit
    scale_factor = np.max(gradmag) / 255
    gradmag = (gradmag / scale_factor).astype(np.uint8)
# Create a binary image of ones where threshold is met, zeros otherwise
    mag_binary = np.zeros_like(gradmag)
    mag_binary[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

# Return the binary image
    return mag_binary


def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi / 2)):
    # Grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # return gray
# Calculate the x and y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    # return sobelx
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # return sobely
# Take the absolute value of the gradient direction,
# apply a threshold, and create a binary image result
    absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    # print(np.max(absgraddir))
    # return absgraddir
    binary_output = np.zeros_like(absgraddir)
    binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1

# Return the binary image
    return binary_output


def hls_select(img, thresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    s_channel = hls[:, :, 2]
    binary_output = np.zeros_like(s_channel)
    binary_output[(s_channel > thresh[0]) & (s_channel <= thresh[1])] = 1
    return binary_output


# image = cv2.imread('test_images/signs_vehicles_xygrad.jpg')
# #image = Image.open('test_images/signs_vehicles_xygrad.jpg')
# print(image.shape)


# # Choose a Sobel kernel size
# ksize = 3 # Choose a larger odd number to smooth gradient measurements

# # Apply each of the thresholding functions
# gradx = abs_sobel_thresh(image, orient='x', sobel_kernel=ksize, thresh=(20, 100))
# grady = abs_sobel_thresh(image, orient='y', sobel_kernel=ksize, thresh=(20, 100))
# mag_binary = mag_thresh(image, sobel_kernel=9, mag_thresh=(30, 100))
# dir_binary = dir_threshold(image, sobel_kernel=15, thresh=(0.7, 1.3))
# hls_binary = hls_select(image,thresh=(50,255))


# combined = np.zeros_like(dir_binary)
# combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1)) & hls_binary == 1] = 1
# color_binary = np.dstack(( np.zeros_like(combined), combined, hls_binary))
# #combined[((gradx == 1) & (grady == 1))] = 1

# combined = np.dstack((combined,combined,combined))*255
# cv2.imwrite('output_images/binary.jpg',combined)

# f, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(1, 5, figsize=(20,10))
# ax1.imshow(gradx,cmap='gray')
# ax2.imshow(grady,cmap='gray')
# ax3.imshow(mag_binary,cmap='gray')
# ax4.imshow(dir_binary,cmap='gray')
# ax5.imshow(color_binary,cmap='gray')
# #plt.imshow(color_binary,cmap='gray')
# plt.show()
