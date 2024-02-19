#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_calibration_parsers/parse.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

class ResizedImageDecompressionNode : public rclcpp::Node
{
public:
    ResizedImageDecompressionNode() : Node("resized_image_decompression_node")
    {
        // Create a subscriber for compressed images
        count = 0;
        compressed_image_subscriber_left = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/left/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_resized_raw_data(msg, true);
            });
        compressed_image_subscriber_right = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/right/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_resized_raw_data(msg, false);
            });
        // Create a publisher for the raw images and camera info
        raw_image_publisher_left = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/left/image_raw", 10);
        raw_image_publisher_right = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/right/image_raw", 10);
    }

private:
    void publish_resized_raw_data(const sensor_msgs::msg::CompressedImage::SharedPtr msg, const bool left){
        cv_bridge::CvImagePtr cv_image_ptr;
        try
        {
            cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Exception: %s", e.what());
            return;
        }
        cv::Mat resized_image;
        cv::resize(cv_image_ptr->image, resized_image, cv::Size(752, 480));
        // Publish the resized decompressed image as a raw image
        if (count == 2){
            raw_image_msg_left.header.stamp = this->get_clock()->now();
            raw_image_msg_left.header.frame_id = "camera_link";
            raw_image_msg_right.header.stamp = raw_image_msg_left.header.stamp;
            raw_image_msg_right.header.frame_id = "camera_link";
            raw_image_publisher_left->publish(raw_image_msg_left);
            raw_image_publisher_right->publish(raw_image_msg_right);
            count = 0;
        }
        if (left){
            cv_bridge::CvImage(cv_image_ptr->header, sensor_msgs::image_encodings::MONO8, resized_image).toImageMsg(raw_image_msg_left);
        }
        else{
            cv_bridge::CvImage(cv_image_ptr->header, sensor_msgs::image_encodings::MONO8, resized_image).toImageMsg(raw_image_msg_right);
        }
        count++; 
    }

    /// ROS publisher object to publish camera info.
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber_left;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber_right;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_publisher_left;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_publisher_right;
    sensor_msgs::msg::CameraInfo camera_info_left;
    sensor_msgs::msg::CameraInfo camera_info_right;
    std::string camera_name_left = "narrow_stereo/left";
    std::string camera_name_right = "narrow_stereo/right";
    sensor_msgs::msg::Image raw_image_msg_left;
    sensor_msgs::msg::Image raw_image_msg_right;
    int count;
    // auto time_stamp;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ResizedImageDecompressionNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
