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
    // Your subscription and publisher setup remains the same
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "stitched_images", 10,
      std::bind(&StereoImageSplitter::split_and_publish, this, std::placeholders::_1));

    left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left_image", 10);
    right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right_image", 10);
  }

private:
  void split_and_publish(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Calculate split dimensions
    int width = cv_ptr->image.cols / 2;
    int height = cv_ptr->image.rows;

    // Split the stitched image into left and right images
    cv::Mat left_image = cv_ptr->image(cv::Rect(0, 0, width, height));
    cv::Mat right_image = cv_ptr->image(cv::Rect(width, 0, width, height));

    // Prepare left and right images for publishing
    // Ensure encoding remains the same
    cv_bridge::CvImage left_msg;
    left_msg.header = msg->header; // Copy header to maintain frame ID and timestamp
    left_msg.encoding = cv_ptr->encoding;
    left_msg.image = left_image;

    cv_bridge::CvImage right_msg;
    right_msg.header = msg->header; // Copy header to maintain frame ID and timestamp
    right_msg.encoding = cv_ptr->encoding;
    right_msg.image = right_image;

    // Publish the images
    left_publisher_->publish(*left_msg.toImageMsg());
    right_publisher_->publish(*right_msg.toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(StereoImageSplitter)
