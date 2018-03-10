// TODO: add includes, etc.
#include "perception/crop.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"

// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
  Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

  void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);
    std::cout << "Cloud based on link: " << msg.header.frame_id << std::endl;
    ROS_INFO("Got point cloud with %ld points", cloud->size());

    // crop the points
    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    double min_x, min_y, min_z, max_x, max_y, max_z;
    ros::param::param("crop_min_x", min_x, 0.6);
    ros::param::param("crop_min_y", min_y, 0.0);
    ros::param::param("crop_min_z", min_z, 0.5);
    ros::param::param("crop_max_x", max_x, 1.2);
    ros::param::param("crop_max_y", max_y, 0.55);
    ros::param::param("crop_max_z", max_z, 1.5);
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);
    ROS_INFO("Cropped to %ld points", cropped_cloud->size());

    // downsample the points
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cropped_cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.01);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);
    ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*downsampled_cloud, msg_out);
    msg_out.header.frame_id = msg.header.frame_id;
    pub_.publish(msg_out); 
  }
}
