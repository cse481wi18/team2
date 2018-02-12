#include "perception/crop.h"
#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo_cropper");
  ros::NodeHandle nh;
  ros::Publisher crop_pub =
    nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  perception::Downsampler downsampler();
  ros::Subscriber sub =
    nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  // ros::Subscriber down_sub = 
  //   nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);
  ros::spin();
  return 0;
}
