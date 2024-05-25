#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher& pub) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

        // Log the image encoding and number of channels
        //ROS_INFO("Image encoding: %s", msg->encoding.c_str());
        //ROS_INFO("Number of channels: %d", cv_ptr->image.channels());

        // Check if the image is single channel
        if (cv_ptr->image.channels() == 1) {
            //ROS_INFO("Image is single channel");
            cv_ptr->encoding = "mono8";
            //ROS_INFO("Image encoding set to mono8");
        } else {
            //ROS_INFO("Image is not single channel");
            // Convert to grayscale explicitly if it's not already single channel
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
            cv_ptr->image = gray_image;
            cv_ptr->encoding = "mono8";
            //ROS_INFO("Converted image to mono8");
        }

        // Set the header of the modified image to be the same as the input message
        cv_ptr->header = msg->header;

        // Publish the modified image
        pub.publish(cv_ptr->toImageMsg());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_subscriber_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image_mono", 10);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 10,
                                                   boost::bind(imageCallback, _1, boost::ref(pub)));

    ROS_INFO("Image subscriber node started");
    ros::spin();

    return 0;
}
