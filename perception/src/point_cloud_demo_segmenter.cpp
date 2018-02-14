#include "perception/crop.h"
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


  ros::Publisher crop_pub =
    nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);

  ros::Publisher above_surface_pub =
      nh.advertise<sensor_msgs::PointCloud2>("above_surface_cloud", 1, true);

  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

  perception::Segmenter segmenter(table_pub, marker_pub, above_surface_pub);
  ros::Subscriber sub2 =
      nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);
    
  ros::spin();
  return 0;
}
