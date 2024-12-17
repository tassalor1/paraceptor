import cv2
import numpy as np

# GStreamer pipeline for 640x480 resolution
gstreamer_pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1 ! "
    "nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Failed to open camera")
    exit()

# Load calibration data
camera_matrix = np.load('camera_matrix.npy')
dist_coefficients = np.load('dist_coefficients.npy')

# Target resolution: 640x480
new_width, new_height = 640, 480

# Scale the camera matrix to 640x480
original_width, original_height = 1920, 1080
scale_x = new_width / original_width
scale_y = new_height / original_height

scaled_camera_matrix = camera_matrix.copy()
scaled_camera_matrix[0, 0] *= scale_x  # fx
scaled_camera_matrix[1, 1] *= scale_y  # fy
scaled_camera_matrix[0, 2] *= scale_x  # cx
scaled_camera_matrix[1, 2] *= scale_y  # cy

# Precompute undistortion maps
map1, map2 = cv2.initUndistortRectifyMap(
    scaled_camera_matrix, dist_coefficients, None, scaled_camera_matrix, (new_width, new_height), cv2.CV_16SC2
)

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    # Apply undistortion
    undistorted_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Show the undistorted frame
    cv2.imshow('Undistorted Frame', undistorted_frame)

    # Save undistorted images when 'c' is pressed
    key = cv2.waitKey(1)
    if key == ord('c'):
        cv2.imwrite(f'depth_calibration_image_{count}.png', undistorted_frame)
        print(f"Saved depth_calibration_image_{count}.png")
        count += 1
    elif key == ord('q'):  # Quit on 'q'
        break

cap.release()
cv2.destroyAllWindows()


