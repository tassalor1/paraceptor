import cv2
import numpy as np

camera_matrix = np.load('camera_matrix.npy')
dist_coefficients = np.load('dist_coefficients.npy')


image = cv2.imread('calibration_image_0.png')  
h, w = image.shape[:2]

# Get the optimal new camera matrix
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix, dist_coefficients, (w, h), 1, (w, h)
)

# Undistort the image
undistorted_image = cv2.undistort(image, camera_matrix, dist_coefficients, None, new_camera_matrix)

x, y, w, h = roi
if roi != (0, 0,0,0):
    undistorted_image = undistorted_image[y:y+h, x:x+w]


# Resize images to match heights
height = min(image.shape[0], undistorted_image.shape[0])
image_resized = cv2.resize(image, (int(image.shape[1] * height / image.shape[0]), height))
undistorted_resized = cv2.resize(undistorted_image, (int(undistorted_image.shape[1] * height / undistorted_image.shape[0]), height))

# Concatenate images horizontally
numpy_horizontal_concat = np.concatenate((image_resized, undistorted_resized), axis=1)

# Scale down the final concatenated image
scale_percent = 50  
final_width = int(numpy_horizontal_concat.shape[1] * scale_percent / 100)
final_height = int(numpy_horizontal_concat.shape[0] * scale_percent / 100)
small_window = cv2.resize(numpy_horizontal_concat, (final_width, final_height))

# Show the resized window
cv2.imshow('Original (Left) vs Undistorted (Right)', small_window)

cv2.waitKey(0)
cv2.destroyAllWindows()
