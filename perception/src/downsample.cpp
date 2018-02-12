// TODO: add includes, etc.
#include "perception/downsample.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
  Downsampler::Downsampler() {}

  void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);
    ROS_INFO("Got point cloud");
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.01);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);
  }
}
