#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo_segmenter");
  ros::NodeHandle nh;
  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  perception::Segmenter segmenter(table_pub, marker_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter);
  ros::spin();
  return 0;
}
