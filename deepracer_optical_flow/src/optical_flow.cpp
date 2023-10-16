#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace std;

class OpticalFlowNode : public rclcpp::Node
{
public:
    OpticalFlowNode() : Node("optical_flow_node")
    {
        // Subscribe to the image_raw topic
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "camera_pkg/left/image_raw", 1, std::bind(&OpticalFlowNode::imageCallback, this, std::placeholders::_1));

        // Create a window to display the optical flow
        cv::namedWindow("Optical Flow");
        
        canvas_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        // Initialize variables
        first_frame_ = true;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    bool first_frame_;
    cv::Mat prev_frame_;
    cv::Mat canvas_;
    std::vector<cv::Point2f> prev_points_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat frame_rgb = frame;
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

        if (first_frame_)
        {
            // Initial frame, detect feature points
            cv::goodFeaturesToTrack(frame, prev_points_, 100, 0.1, 10);
            prev_frame_ = frame.clone();
            first_frame_ = false;
        }
        else
        {
            // Calculate optical flow
            std::vector<cv::Point2f> next_points;
            std::vector<uchar> status;
            std::vector<float> err;
            cv::TermCriteria criteria = cv::TermCriteria(
                cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.03);

            cv::calcOpticalFlowPyrLK(prev_frame_, frame, prev_points_, next_points, status, err, cv::Size(21, 21), 3, criteria);

            // Draw the optical flow vectors on the frame
            for (size_t i = 0; i < prev_points_.size(); ++i)
            {
                if (status[i])
                {
                    // cv::line(canvas_, prev_points_[i], next_points[i], cv::Scalar(255, 255, 0), 2);
                    cv::line(frame_rgb, prev_points_[i], next_points[i], cv::Scalar(255, 255, 0), 2);
                }
            }

            // Update the previous frame and points for the next iteration
            prev_frame_ = frame.clone();
            prev_points_ = next_points;
            first_frame_ = true;
            // cv::goodFeaturesToTrack(frame, prev_points_, 1000, 0.01, 10);

        }
        cv::Mat combinedImage;
        cv::hconcat(canvas_, frame_rgb, combinedImage);
        cv::imshow("Optical Flow", combinedImage);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpticalFlowNode>());
    rclcpp::shutdown();
    return 0;
}
