// Filename: stereo_image_splitter.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class StereoImageSplitter : public rclcpp::Node
{
public:
  // Update the constructor to accept NodeOptions
  explicit StereoImageSplitter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stereo_image_splitter", options)
  {
    this->declare_parameter<bool>("is_grey", false);
    this->get_parameter("is_grey", is_grey_);

    // Your subscription and publisher setup remains the same
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "stitched_images", 10,
      std::bind(&StereoImageSplitter::split_and_publish, this, std::placeholders::_1));

    left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left_image", 10);
    right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right_image", 10);
  }

private:
  void split_and_publish(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Direct conversion to cv::Mat with cv_bridge handling common encodings
            cv_ptr = cv_bridge::toCvCopy(msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Check source encoding and convert if necessary
        cv::Mat processed_image;
        if (msg->encoding == "yuv422_yuy2") {
            // Convert YUV422 (YUY2) to BGR for processing
            cv::cvtColor(cv_ptr->image, processed_image, cv::COLOR_YUV2BGR_YUY2);
        } else if (msg->encoding != "bgr8" && !is_grey_) {
            // Assume the source is BGR if not grayscale and not YUY2. Adjust if your camera uses RGB.
            processed_image = cv_ptr->image.clone();
        }

        // Additional check for grayscale conversion
        if (is_grey_) {
            if (processed_image.channels() > 1) {
                // Convert BGR or RGB to Grayscale
                cv::cvtColor(processed_image, processed_image, cv::COLOR_BGR2GRAY);
            }
        } else if (processed_image.empty()) {
            // For cases where the encoding is not 'yuv422_yuy2' but still color
            processed_image = cv_ptr->image.clone();
        }

        // Split the image into left and right halves
        int width = processed_image.cols / 2;
        int height = processed_image.rows;

        cv::Mat right_image = processed_image(cv::Rect(0, 0, width, height));
        cv::Mat left_image = processed_image(cv::Rect(width, 0, width, height));

        // Set the appropriate encoding for publishing
        std::string encoding = is_grey_ ? "mono8" : "bgr8";

        // Publish the left and right images
        cv_bridge::CvImage left_msg(cv_ptr->header, encoding, left_image);
        cv_bridge::CvImage right_msg(cv_ptr->header, encoding, right_image);

        left_publisher_->publish(*left_msg.toImageMsg());
        right_publisher_->publish(*right_msg.toImageMsg());
    }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
  bool is_grey_; // Flag indicating if the input images are grayscale
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(StereoImageSplitter)
