import sys
import cv2 
import imutils
import os
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(parent_dir)

from yolov5.yoloDet import YoloTRT

# you need picture of known object w/ height and distance from camera to object- picture object part of coco dataset 

# use path for library and engine file
library = os.path.join(parent_dir, "yolov5", "build", "libmyplugins.so")
engine = os.path.join(parent_dir, "yolov5", "build", "yolov5s.engine")
model = YoloTRT(library=library, engine=engine, conf=0.5, yolo_ver="v5")

# use undistored pciture of the known object
image = cv2.imread("depth_calibration_image_4.png")

# this is the object class for the known object in the picture
class_object = 'suitcase'

focal_object_distance = 52
object_height = 15

detections, t = model.Inference(image)
print(detections)
class_detections = [det for det in detections if det['class'] == class_object]
class_boxes = [box['box'] for box in class_detections]

bounding_box = class_boxes[0]
bounding_box_height = bounding_box[3] - bounding_box[1]

focal_length = (bounding_box_height * focal_object_distance) / object_height

depth_estimation = (focal_length * object_height) / bounding_box_height
print(f'depth_estimation: {depth_estimation}')
cv2.imshow("Output",image)
key = cv2.waitKey(1)
cv2.imwrite(f'focal_calc_pic.png', image)
 #if key == ord('q'):
     #break
cv2.destroyAllWindows()
