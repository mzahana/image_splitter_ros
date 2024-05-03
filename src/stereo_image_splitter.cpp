#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class StereoImageSplitter
{
public:
    explicit StereoImageSplitter(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh), nhp_(nhp), it_(nh_)
    {
        // Initialize parameters
        nhp_.param("is_grey", is_grey_, false);

        // Set up subscriber and publishers using image_transport
        image_sub_ = it_.subscribe("stitched_images", 1, &StereoImageSplitter::split_and_publish, this);
        left_pub_ = it_.advertise("left_image", 1);
        right_pub_ = it_.advertise("right_image", 1);
    }

private:
    void split_and_publish(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Check source encoding and convert if necessary
        cv::Mat processed_image;
        if (msg->encoding == "yuv422_yuy2") {
            // Convert YUV422 (YUY2) to BGR for processing
            cv::cvtColor(cv_ptr->image, processed_image, cv::COLOR_YUV2BGR_YUY2);
        } else if (msg->encoding == "rgb8") {
            // Assume the source is BGR if not grayscale and not YUY2. Adjust if your camera uses RGB.
            // processed_image = cv_ptr->image.clone();
            cv::cvtColor(cv_ptr->image, processed_image, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "rgba8") {
            // Assume the source is BGR if not grayscale and not YUY2. Adjust if your camera uses RGB.
            // processed_image = cv_ptr->image.clone();
            cv::cvtColor(cv_ptr->image, processed_image, cv::COLOR_RGBA2BGR);
        } else{
            processed_image = cv_ptr->image.clone();
        }

        // Additional check for grayscale conversion
        if (is_grey_) {
            if (processed_image.channels() > 1) {
                // Convert BGR  to Grayscale
                cv::cvtColor(processed_image, processed_image, cv::COLOR_BGR2GRAY);
            }
        } else if (processed_image.empty()) {
            // For cases where the encoding is not 'yuv422_yuy2' but still color
            processed_image = cv_ptr->image.clone();
        }

        int width = processed_image.cols / 2;
        int height = processed_image.rows;
        cv::Mat right_image = processed_image(cv::Rect(0, 0, width, height));
        cv::Mat left_image = processed_image(cv::Rect(width, 0, width, height));

        // Publish the split images
        cv_bridge::CvImage left_msg;
        left_msg.header = msg->header;
        left_msg.encoding = is_grey_ ? "mono8" : "bgr8";
        left_msg.image = left_image;
        left_pub_.publish(left_msg.toImageMsg());

        cv_bridge::CvImage right_msg;
        right_msg.header = msg->header;
        right_msg.encoding = is_grey_ ? "mono8" : "bgr8";
        right_msg.image = right_image;
        right_pub_.publish(right_msg.toImageMsg());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher left_pub_;
    image_transport::Publisher right_pub_;
    bool is_grey_;
};
