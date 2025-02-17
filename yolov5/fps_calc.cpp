#include <iostream>
#include <opencv2/opencv.hpp>
#include <cnpy.h>
#include "yolov5_det.h"
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

using namespace cv;
using namespace std;


int main() {
    // GStreamer pipeline
    string g_streamer_pipeline = 
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1 ! "
        "nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink";

    // Load YOLO detector
    fs::path dir("build");
    fs::path file("yolov5s.engine");
    fs::path engine_path = dir / file;
    YoloDetector detector(engine_path.string());

    // Open camera
    VideoCapture cap(g_streamer_pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Failed to open camera" << endl;
        return -1;
    }
    cout << "Camera opened successfully" << endl;

    // Load calibration data
    cnpy::NpyArray cam_matrix_npy = cnpy::npy_load("../camera_calibration/camera_matrix.npy");
    cnpy::NpyArray dist_coef_npy = cnpy::npy_load("../camera_calibration/dist_coeffs.npy");

    // Convert to OpenCV Mat
    Mat cam_matrix = Mat(3, 3, CV_64F, cam_matrix_npy.data<double>());
    Mat dist_coef = Mat(1, 5, CV_64F, dist_coef_npy.data<double>());

    // Scale camera matrix for resized frames
    int new_width = 640, new_height = 480;
    int original_width = 1920, original_height = 1080;

    double scale_x = static_cast<double>(new_width) / original_width;
    double scale_y = static_cast<double>(new_height) / original_height;

    Mat scaled_camera_matrix = cam_matrix.clone();
    scaled_camera_matrix.at<double>(0, 0) *= scale_x;  // fx
    scaled_camera_matrix.at<double>(1, 1) *= scale_y;  // fy
    scaled_camera_matrix.at<double>(0, 2) *= scale_x;  // cx
    scaled_camera_matrix.at<double>(1, 2) *= scale_y;  // cy

    // Compute undistortion maps
    Mat map1, map2;
    initUndistortRectifyMap(cam_matrix, dist_coef, Mat(), scaled_camera_matrix,
                            Size(new_width, new_height), CV_16SC2, map1, map2);

    // Main loop
    Mat frame;
    while (cap.read(frame)) {
        // Undistort frame
        Mat undistorted_frame;
        remap(frame, undistorted_frame, map1, map2, INTER_LINEAR);

        // Run detection
        auto detections = detector.process(undistorted_frame);

        // Draw bounding boxes
        for (const auto& det : detections) {
            Rect rect(
                static_cast<int>(det.bbox[0] - det.bbox[2]/2),  // x
                static_cast<int>(det.bbox[1] - det.bbox[3]/2),  // y
                static_cast<int>(det.bbox[2]),                   // width
                static_cast<int>(det.bbox[3])                    // height
            );
            rectangle(undistorted_frame, rect, Scalar(0, 255, 0), 2);
        }

        // Display result
        imshow("Detected", undistorted_frame);
        if (waitKey(1) == 27) break;  // ESC to exit
    }

    return 0;
}
