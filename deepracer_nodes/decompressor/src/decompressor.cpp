#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

class ImageDecompressionNode : public rclcpp::Node
{
public:
    ImageDecompressionNode() : Node("image_decompression_node")
    {
        // Create a subscriber for compressed images
        compressed_image_subscriber_left = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/left/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_raw_data_left(msg);
            });
        compressed_image_subscriber_right = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_pkg/right/compressed_image", 10,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                this->publish_raw_data_right(msg);
            });
        // Create a publisher for the raw image
        raw_image_publisher_left = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/left/image_raw", 10);
        raw_image_publisher_right = this->create_publisher<sensor_msgs::msg::Image>("camera_pkg/right/image_raw", 10);
    }

private:
    void publish_raw_data_left(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_image_ptr;
        try
        {
            cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Exception: %s", e.what());
            return;
        }

        // Publish the decompressed image as a raw image
        sensor_msgs::msg::Image raw_image_msg;
        cv_image_ptr->toImageMsg(raw_image_msg);
        raw_image_msg.header.stamp = msg->header.stamp;
        raw_image_msg.header.frame_id = "camera_link";
        raw_image_publisher_left->publish(raw_image_msg);
    }
    void publish_raw_data_right(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_image_ptr;
        try
        {
            cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Exception: %s", e.what());
            return;
        }

        // Publish the decompressed image as a raw image
        sensor_msgs::msg::Image raw_image_msg;
        cv_image_ptr->toImageMsg(raw_image_msg);
        raw_image_msg.header.stamp = msg->header.stamp;
        raw_image_msg.header.frame_id = "camera_link_right";
        raw_image_publisher_right->publish(raw_image_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber_left;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber_right;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_publisher_left;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_publisher_right;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageDecompressionNode>());
    rclcpp::shutdown();
    return 0;
}
