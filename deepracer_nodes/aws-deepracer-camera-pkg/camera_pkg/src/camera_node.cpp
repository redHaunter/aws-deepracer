///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "deepracer_interfaces_pkg/srv/video_state_srv.hpp"
#include "deepracer_interfaces_pkg/msg/camera_msg.hpp"
#include "opencv2/opencv.hpp"
// #include "camera_info_manager/camera_info_manager.h"
#include "camera_calibration_parsers/parse.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <thread>
#include <atomic>
#include <memory>

namespace MediaEng
{
#define DEFAULT_IMAGE_WIDTH 160
#define DEFAULT_IMAGE_HEIGHT 120
    class CameraNode : public rclcpp::Node
    {
        /// This class creates the camera_node responsible to read data from the cameras
        /// and publish it as images.
    public:
        const char *CAMERA_MSG_TOPIC = "video_mjpeg";
        const char *DISPLAY_MSG_TOPIC_1 = "left/image_raw";
        const char *DISPLAY_MSG_TOPIC_2 = "right/image_raw";
        const char *ACTIVATE_CAMERA_SERVICE_NAME = "media_state";

        /// @param node_name Name of the node to be created.
        /// @param cameraIdxList List of camera indexes to iterate over to find the valid
        ///                      indices where the camera is connected.
        CameraNode(const std::string &node_name, const std::vector<int> cameraIdxList)
            : Node(node_name),
              produceFrames_(false),
              resizeImages_(true)
        {
            RCLCPP_INFO(this->get_logger(), "%s started", node_name.c_str());
            this->declare_parameter<bool>("resize_images", resizeImages_);
            // Update resizeImages boolean based on the parameter
            resizeImages_ = this->get_parameter("resize_images").as_bool();

            // Scan and load only valid streamers to Video Capture list.
            scanCameraIndex(cameraIdxList);

            image_publisher_1 = this->create_publisher<sensor_msgs::msg::CompressedImage>("left/image_compress", 1);
            image_publisher_2 = this->create_publisher<sensor_msgs::msg::CompressedImage>("right/image_compress", 1);

            cameraInfoPub_1 = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
            cameraInfoPub_2 = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);

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
        rclcpp::TimerBase::SharedPtr timer_;
        /// Scan camera index to add valid streamers to Video Capture List.
        /// @param cameraIdxList List of camera indexes to iterate over to find the valid
        ///                      indices where the camera is connected.
        void scanCameraIndex(const std::vector<int> cameraIdxList)
        {
            for (auto idx : cameraIdxList)
            {
                RCLCPP_INFO(this->get_logger(), "Scanning Camera Index %d ", idx);
                auto cap = cv::VideoCapture(idx, cv::CAP_V4L);
                cv::Mat test_frame;
                cap >> test_frame;
                if (test_frame.empty() || !cap.isOpened())
                {
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
            switch (videoIndexList_.size())
            {
            case 0:
                RCLCPP_ERROR(this->get_logger(), "[Camera Package] No cameras detected.");
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "[Camera Package] Single Camera Detected at index:  %d", videoIndexList_[0]);
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "[Camera Package] Stereo Cameras Detected at indexes:  %d, %d", videoIndexList_[0], videoIndexList_[1]);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "[Camera Package] Error while detecting cameras.");
            }
        }
        /// Request handler for the media server.
        void videoProducerStateHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<deepracer_interfaces_pkg::srv::VideoStateSrv::Request> req,
                                   std::shared_ptr<deepracer_interfaces_pkg::srv::VideoStateSrv::Response> res)
        {
            (void)request_header;
            // Stop the stream on request.
            produceFrames_ = false;
            if (videoWorker_.joinable())
            {
                videoWorker_.join();
            }
            int i = 0;
            // Check that we are collecting from the mjpeg channel.
            for (auto &cap : videoCaptureList_)
            {
                if (!cap.isOpened() || cap.get(cv::CAP_PROP_FOURCC) != cv::VideoWriter::fourcc('M', 'J', 'P', 'G'))
                {
                    RCLCPP_ERROR(this->get_logger(), "Unable to get MJPEG stream: %d", videoIndexList_[i]);
                    res->error = 1;
                    return;
                }
                ++i;
            }
            if (req->activate_video)
            {
                // timer_ = this->create_wall_timer(
                //     std::chrono::milliseconds(40),
                //     std::bind(&MediaEng::CameraNode::produceFrames, this));
                produceFrames_ = true;
                videoWorker_ = std::thread([&]
                                           { produceFrames(); });
            }
            res->error = 0;
        }

        /// Collects the frame from the camera and publishes them. This method is intended
        /// to run in a separate thread.
        void produceFrames()
        {
            while (produceFrames_)
            {
                deepracer_interfaces_pkg::msg::CameraMsg msg;
                sensor_msgs::msg::CameraInfo camera_info_left;
                sensor_msgs::msg::CameraInfo camera_info_right;
                std::string camera_name_left = "narrow_stereo/left";
                std::string camera_name_right = "narrow_stereo/right";
                camera_calibration_parsers::readCalibration(
                    "/home/deepracer/stereo_calibration/left.yaml", camera_name_left, camera_info_left);
                camera_calibration_parsers::readCalibration(
                    "/home/deepracer/stereo_calibration/right.yaml", camera_name_right, camera_info_right);
                bool left = true;
                cv::Mat frame_left;
                cv::Mat frame_right;
                for (auto &cap : videoCaptureList_)
                {
                    if (!cap.isOpened())
                    {
                        continue;
                    }
                    cv::Mat frame;
                    cap >> frame;
                    if (left)
                    {
                        cap >> frame_left;
                        left = false;
                    }
                    else
                    {
                        cap >> frame_right;
                    }
                    if (frame.empty())
                    {
                        RCLCPP_ERROR(this->get_logger(), "No frame returned. Check if camera is plugged in correctly.");
                        continue;
                    }
                    try
                    {
                        if (resizeImages_)
                        {
                            cv::resize(frame, frame, cv::Size(DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT));
                        }
                        // msg.images.push_back(*(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg().get()));
                    }
                    catch (cv_bridge::Exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                        produceFrames_ = false;
                        return;
                    }
                }
                try
                {
                    camera_info_left.header.stamp = this->get_clock()->now();
                    camera_info_right.header.stamp = camera_info_left.header.stamp;

                    std::vector<int> compression_params;
                    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                    compression_params.push_back(10);

                    cv::imencode(".jpg", frame_left, jpeg_data_left, compression_params);
                    cv::imencode(".jpg", frame_right, jpeg_data_right, compression_params);

                    sensor_msgs::msg::CompressedImage msg_left;
                    sensor_msgs::msg::CompressedImage msg_right;

                    msg_left.header.stamp = camera_info_left.header.stamp;
                    msg_right.header.stamp = camera_info_left.header.stamp;
                    msg_left.format = "jpeg";
                    msg_right.format = "jpeg";
                    msg_left.data = jpeg_data_left;
                    msg_right.data = jpeg_data_right;

                    switch (videoIndexList_.size())
                    {
                    case 1:
                        image_publisher_1->publish(msg_left);
                        cameraInfoPub_1->publish(camera_info_left);
                        break;
                    case 2:
                        image_publisher_1->publish(msg_left);
                        image_publisher_2->publish(msg_right);
                        cameraInfoPub_1->publish(camera_info_left);
                        cameraInfoPub_2->publish(camera_info_right);
                        break;
                    default:
                        break;
                    }
                }
                catch (const std::exception &ex)
                {
                    RCLCPP_ERROR(this->get_logger(), "Publishing camera images to topics failed %s", ex.what());
                }
            }
        }

        /// ROS publisher object to the publish camera images to camera message topic.
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::CameraMsg>::SharedPtr videoPub_;
        /// ROS publisher object to the publish camera images to display message topic.
        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr displayPub_1;
        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr displayPub_2;

        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_1;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_2;
        std::vector<uint8_t> jpeg_data_left;
        std::vector<uint8_t> jpeg_data_right;
        /// ROS publisher object to the publish camera info.
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub_1;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub_2;
        /// ROS service object to activate the camera to publish images.
        rclcpp::Service<deepracer_interfaces_pkg::srv::VideoStateSrv>::SharedPtr activateCameraService_;
        /// Boolean for starting and stopping the worker thread.
        std::atomic<bool> produceFrames_;
        /// Boolean to resize images.
        std::atomic<bool> resizeImages_;
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //! TODO Remove hardcode after testing.
    // TODO Update not to log uvcvideo: Buffer is NULL in /var/log/syslog

    // Earlier logic of having Left/Center/Right map to index 4/1/0 doesn't work in Ubuntu 20.04
    // The camera indexes keep changing intermittently in Ubuntu 20.04.
    // Hence modifying the logic to scan for indexes in descending order and add to the Video Capture list of the valid capture elements.
    // In case of Stereo Cameras: The index with greater number represents Left Camera.
    std::vector<int> cameraIndex{4, 3, 2, 1, 0};
    // Create the camera_node.
    rclcpp::spin(std::make_shared<MediaEng::CameraNode>("camera_node", cameraIndex));
    rclcpp::shutdown();
    return 0;
}
