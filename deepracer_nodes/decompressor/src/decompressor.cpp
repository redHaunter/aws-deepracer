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

class ImageDecompressionNode : public rclcpp::Node
{
public:
    ImageDecompressionNode() : Node("image_decompression_node"), publishRGB(false)
    {
        this->declare_parameter<bool>("publish_rgb", publishRGB);
        // Update resizeImages boolean based on the parameter
        publishRGB = this->get_parameter("publish_rgb").as_bool();
        count = 0;
        // Create a subscriber for compressed images
        camera_calibration_parsers::readCalibration(
            "/home/redha/ros2_ws/src/aws-deepracer/deepracer_nodes/decompressor/config/left.yaml", camera_name_left, camera_info_left);
        camera_calibration_parsers::readCalibration(
            "/home/redha/ros2_ws/src/aws-deepracer/deepracer_nodes/decompressor/config/right.yaml", camera_name_right, camera_info_right);
        raw_image_msg_left.header.frame_id = "camera_link_left";
        raw_image_msg_right.header.frame_id = "camera_link_right";
        camera_info_left.header.frame_id = "camera_link_left";
        camera_info_right.header.frame_id = "camera_link_right";
        compressed_image_subscriber_left = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/left/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_raw_data(msg, true);
            });
        compressed_image_subscriber_right = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/right/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_raw_data(msg, false);
            });
        // Create a publisher for the raw images and camera info
        raw_image_publisher_left = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/left/image_raw", 10);
        raw_image_publisher_right = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/right/image_raw", 10);
        cameraInfoPub_1 = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_pkg/left/camera_info", 10);
        cameraInfoPub_2 = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_pkg/right/camera_info", 10);
    }

private:
    void publish_raw_data(const sensor_msgs::msg::CompressedImage::SharedPtr msg, const bool left)
    {
        cv_bridge::CvImagePtr cv_image_ptr;
        try
        {
            if (publishRGB)
                cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            else 
                cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Exception: %s", e.what());
            return;
        }
        // Publish decompressed images as a raw image
        if (count == 2)
        {
            raw_image_msg_left.header.stamp = this->get_clock()->now();
            raw_image_msg_right.header.stamp = raw_image_msg_left.header.stamp;
            camera_info_left.header.stamp = raw_image_msg_left.header.stamp;
            camera_info_right.header.stamp = raw_image_msg_left.header.stamp;
            raw_image_publisher_left->publish(raw_image_msg_left);
            raw_image_publisher_right->publish(raw_image_msg_right);
            cameraInfoPub_1->publish(camera_info_left);
            cameraInfoPub_2->publish(camera_info_right);
            count = 0;
        }
        if (left)
        {
            cv_image_ptr->toImageMsg(raw_image_msg_left);
        }
        else
        {
            cv_image_ptr->toImageMsg(raw_image_msg_right);
        }
        count++;
    }
    /// ROS publisher object to the publish camera info.
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub_1;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub_2;
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
    std::atomic<bool> publishRGB;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ImageDecompressionNode>();
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(std::make_shared<ImageDecompressionNode>());
    rclcpp::shutdown();
    return 0;
}
