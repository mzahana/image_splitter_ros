#include "ros/ros.h"
#include "stereo_image_splitter.cpp" // Include the splitter header

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_image_splitter");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    StereoImageSplitter node(nh, nhp); // Assuming your class constructor accepts a NodeHandle

    ros::spin();
    return 0;
}
