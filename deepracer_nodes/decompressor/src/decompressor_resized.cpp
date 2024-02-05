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
        compressed_image_subscriber_left = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/left/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_resized_raw_data_left(msg);
            });
        compressed_image_subscriber_right = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/right/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_resized_raw_data_right(msg);
            });
        // Create a publisher for the raw images and camera info
        raw_image_publisher_left = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/left/image_raw", 10);
        raw_image_publisher_right = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/right/image_raw", 10);
        cameraInfoPub_1 = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_pkg/left/camera_info", 10);
        cameraInfoPub_2 = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_pkg/right/camera_info", 10);
    }

private:
    void publish_resized_raw_data_left(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
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
        camera_calibration_parsers::readCalibration(
            "/home/redha/ros2_ws/src/aws-deepracer/deepracer_nodes/decompressor/config/left.yaml", camera_name_left, camera_info_left);
        camera_info_left.header.stamp = this->get_clock()->now();
        camera_info_left.header.frame_id = "camera_link";
        // Resize the image to 130% of the current size
        cv::Mat resized_image;
        cv::resize(cv_image_ptr->image, resized_image, cv::Size(752, 480));
        // Publish the resized decompressed image as a raw image
        sensor_msgs::msg::Image raw_image_msg;
        cv_bridge::CvImage(cv_image_ptr->header, sensor_msgs::image_encodings::MONO8, resized_image).toImageMsg(raw_image_msg);
        raw_image_msg.header.stamp = camera_info_left.header.stamp;
        raw_image_msg.header.frame_id = "camera_link";
        raw_image_publisher_left->publish(raw_image_msg);
        cameraInfoPub_1->publish(camera_info_left);
    }

    void publish_resized_raw_data_right(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
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
        camera_calibration_parsers::readCalibration(
            "/home/redha/ros2_ws/src/aws-deepracer/deepracer_nodes/decompressor/config/right.yaml", camera_name_right, camera_info_right);
        camera_info_right.header.stamp = this->get_clock()->now();
        camera_info_right.header.frame_id = "camera_link";
        // Resize the image to 130% of the current size
        cv::Mat resized_image;
        cv::resize(cv_image_ptr->image, resized_image, cv::Size(752, 480));
        // Publish the resized decompressed image as a raw image
        sensor_msgs::msg::Image raw_image_msg;
        cv_bridge::CvImage(cv_image_ptr->header, sensor_msgs::image_encodings::MONO8, resized_image).toImageMsg(raw_image_msg);
        raw_image_msg.header.stamp = camera_info_right.header.stamp;
        raw_image_msg.header.frame_id = "camera_link";
        raw_image_publisher_right->publish(raw_image_msg);
        cameraInfoPub_2->publish(camera_info_right);
    }

    /// ROS publisher object to publish camera info.
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ResizedImageDecompressionNode>();
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(std::make_shared<ImageDecompressionNode>());
    rclcpp::shutdown();
    return 0;
}
