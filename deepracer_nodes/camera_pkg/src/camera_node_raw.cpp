#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "deepracer_interfaces_pkg/srv/video_state_srv.hpp"
#include "deepracer_interfaces_pkg/msg/camera_msg.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"

#include <thread>
#include <atomic>
#include <memory>

namespace MediaEng {
    #define DEFAULT_IMAGE_WIDTH 160
    #define DEFAULT_IMAGE_HEIGHT 120
    class CameraNode : public rclcpp::Node
    {
    /// This class creates the camera_node responsible to read data from the cameras
    /// and publish it as images.
    public:
        // const char* CAMERA_MSG_TOPIC = "video_mjpeg";
        const char *DISPLAY_MSG_TOPIC_1 = "left/image_raw";
        const char *DISPLAY_MSG_TOPIC_2 = "right/image_raw";
        const char* ACTIVATE_CAMERA_SERVICE_NAME = "media_state";

        /// @param node_name Name of the node to be created.
        /// @param cameraIdxList List of camera indexes to iterate over to find the valid
        ///                      indices where the camera is connected.
        CameraNode(const std::string & node_name, const std::vector<int> cameraIdxList)
        : Node(node_name),
        produceFrames_(false),
        resizeImages_(true),
        jpegQuality_(50)
        {
            RCLCPP_INFO(this->get_logger(), "%s started", node_name.c_str());
            this->declare_parameter<bool>("resize_images", resizeImages_);
            this->declare_parameter<int>("jpeg_quality", jpegQuality_);
            // Update resizeImages boolean based on the parameter
            resizeImages_ = this->get_parameter("resize_images").as_bool();
            jpegQuality_ = this->get_parameter("jpeg_quality").as_int();

            // Scan and load only valid streamers to Video Capture list.
            scanCameraIndex(cameraIdxList);

            image_publisher_1 = this->create_publisher<sensor_msgs::msg::CompressedImage>("left/compressed_image", 1);
            image_publisher_2 = this->create_publisher<sensor_msgs::msg::CompressedImage>("right/compressed_image", 1);
            
            // Create a service to activate the publish of camera images.
            activateCameraService_ = this->create_service<deepracer_interfaces_pkg::srv::VideoStateSrv>(
                                                                                ACTIVATE_CAMERA_SERVICE_NAME,
                                                                                std::bind(&CameraNode::videoProducerStateHdl,
                                                                                          this,
                                                                                          std::placeholders::_1,
                                                                                          std::placeholders::_2,
                                                                                          std::placeholders::_3));
        }
        ~CameraNode() = default;

    private:
        /// Scan camera index to add valid streamers to Video Capture List.
        /// @param cameraIdxList List of camera indexes to iterate over to find the valid
        ///                      indices where the camera is connected.
        void scanCameraIndex(const std::vector<int> cameraIdxList) {
            for (auto idx : cameraIdxList){
                RCLCPP_INFO(this->get_logger(), "Scanning Camera Index %d ", idx);
                auto cap = cv::VideoCapture(idx, cv::CAP_V4L);
                cv::Mat test_frame;
                cap >> test_frame;
                if(test_frame.empty() || !cap.isOpened()){
                    RCLCPP_ERROR(this->get_logger(), "Unable to create video capture stream on index: %d", idx);
                    continue;
                }
                // Add to valid video capture list
                videoCaptureList_.push_back(cap);
                videoCaptureList_.back().set(cv::CAP_PROP_FOURCC,
                                            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                // Add to valid video index list
                videoIndexList_.push_back(idx);
            }
            switch(videoIndexList_.size()) {
                case 0 :
                    RCLCPP_ERROR(this->get_logger(), "[Camera Package] No cameras detected.");
                    break;
                case 1 :
                    RCLCPP_INFO(this->get_logger(), "[Camera Package] Single Camera Detected at index:  %d", videoIndexList_[0]);
                    break;
                case 2 :
                    RCLCPP_INFO(this->get_logger(), "[Camera Package] Stereo Cameras Detected at indexes:  %d, %d", videoIndexList_[0], videoIndexList_[1]);
                    break;
                default :
                    RCLCPP_ERROR(this->get_logger(), "[Camera Package] Error while detecting cameras.");
            }
        }
        cv::Mat cropCenter(cv::Mat& image, int width, int height) {
            // Calculate the coordinates to define the rectangle for the center crop
            int startX = (image.cols - width) / 2;
            int startY = (image.rows - height) / 2;

            // Define the rectangle for the center crop
            cv::Rect cropRegion(startX, startY, width, height);

            // Crop the center part of the image
            cv::Mat croppedImage = image(cropRegion);

            return croppedImage;
        }
        /// Request handler for the media server.
        void videoProducerStateHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<deepracer_interfaces_pkg::srv::VideoStateSrv::Request> req,
                                   std::shared_ptr<deepracer_interfaces_pkg::srv::VideoStateSrv::Response> res) {
            (void)request_header;
            // Stop the stream on request.
            produceFrames_ = false;
            if (videoWorker_.joinable()) {
                videoWorker_.join();
            }
            int i = 0;
            // Check that we are collecting from the mjpeg channel.
            for (auto& cap :  videoCaptureList_) {
                if(!cap.isOpened() || cap.get(cv::CAP_PROP_FOURCC) != cv::VideoWriter::fourcc('M', 'J', 'P', 'G')) {
                    RCLCPP_ERROR(this->get_logger(), "Unable to get MJPEG stream: %d", videoIndexList_[i]);
                    res->error = 1;
                    return;
                }
                ++i;
            }
            if (req->activate_video) {
                produceFrames_ = true;
                videoWorker_ = std::thread([&]{produceFrames();});
            }
            res->error = 0;
        }

        /// Collects the frame from the camera and publishes them. This method is intended
        /// to run in a separate thread.
        void produceFrames() {
            while (produceFrames_) {
                std::vector<cv::Mat> frames;
                for (auto& cap :  videoCaptureList_) {
                    if (!cap.isOpened()) {
                        continue;
                    }
                    cv::Mat frame;
                    cap >> frame;
                    if (frame.empty()) {
                        RCLCPP_ERROR(this->get_logger(), "No frame returned. Check if camera is plugged in correctly.");
                        continue;
                    }
                    try {
                        if(resizeImages_) {
                            int crop_factor = 7;
                            int cropped_height = 480 - (2 * (480 / crop_factor));
                            int cropped_width = 640 - (2 * (640 / crop_factor));
                            frame = cropCenter(frame, cropped_width, cropped_height);
                        }
                        frames.push_back(frame);
                    }
                    catch (cv_bridge::Exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                        produceFrames_ = false;
                        return;
                    }
                }
                try {
                    // image_msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frames.at(0)).toCompressedImageMsg();
                    // image_msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frames.at(1)).toCompressedImageMsg();
                    
                    image_msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frames.at(0)).toCompressedImageMsg();
                    image_msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frames.at(1)).toCompressedImageMsg();
                    
                    image_msg_left->header.stamp = this->get_clock()->now();
                    image_msg_right->header.stamp = this->get_clock()->now();
                    image_msg_left->header.frame_id = "camera_link";
                    image_msg_right->header.frame_id = "camera_link";

                    switch (videoIndexList_.size())
                    {
                    case 1:
                        // image_publisher_1->publish(msg_left);
                        // cameraInfoPub_1->publish(camera_info_left);
                        break;
                    case 2:
                        image_publisher_1->publish(*image_msg_left);
                        image_publisher_2->publish(*image_msg_right);
                        break;
                    default:
                        break;
                    }
                }
                    catch (const std::exception &ex) {
                    RCLCPP_ERROR(this->get_logger(), "Publishing camera images to topics failed %s", ex.what());
                }
            }
        }

        /// ROS publisher object to the publish camera images to camera message topic.
        sensor_msgs::msg::CompressedImage::SharedPtr image_msg_left;
        sensor_msgs::msg::CompressedImage::SharedPtr image_msg_right;
        // image_transport::Publisher image_publisher_1;
        // image_transport::Publisher image_publisher_2;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_1;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_2;
        /// ROS service object to activate the camera to publish images.
        rclcpp::Service<deepracer_interfaces_pkg::srv::VideoStateSrv>::SharedPtr activateCameraService_;
        /// Boolean for starting and stopping the worker thread.
        std::atomic<bool> produceFrames_;
        /// Boolean to resize images.
        std::atomic<bool> resizeImages_;
        std::atomic<int> jpegQuality_;
        /// List of OpenCV video capture object used to retrieve frames from the cameras.
        std::vector<cv::VideoCapture> videoCaptureList_;
        /// List of valid camera indices identified after scanning.
        std::vector<int> videoIndexList_;
        /// Thread that constantly retrieves frames from the camera and publishes the frames.
        std::thread videoWorker_;
        /// Camera index parameter to capture video frames from the specific camera.
        int cameraIndex_;
    };
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    std::vector<int> cameraIndex {4, 3, 2, 1, 0};
    // rclcpp::spin(std::make_shared<MediaEng::CameraNode>("camera_node", cameraIndex));
    auto node = std::make_shared<MediaEng::CameraNode>("camera_node", cameraIndex);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}