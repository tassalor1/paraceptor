#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class CaptureCVData : public rclcpp::Node
{
public: 
 CaptureCVData() : Node("cv_data_subscription")
 {

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera", 
        10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg){
            this->screenshot_data(msg);
        });
 }
    void screenshot_data(const sensor_msgs::msg::Image::SharedPtr image)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = cv_ptr->image;

        cv::imshow("camera_feed", frame);
        cv::waitKey(1);
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CaptureCVData>());
    rclcpp::shutdown();
    return 0;

}