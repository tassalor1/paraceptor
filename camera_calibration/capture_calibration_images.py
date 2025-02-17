import cv2

gstreamer_pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink"
)

cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break
    
    cv2.imshow('Frame', frame)
    
    key = cv2.waitKey(1)
    if key == ord('c'):  # Press 'c' to capture an image
        cv2.imwrite(f'calibration_image_{count}.png', frame)
        print(f"Saved calibration_image_{count}.png")
        count += 1
    elif key == ord('q'):  
        break

cap.release()
cv2.destroyAllWindows()

