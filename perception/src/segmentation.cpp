#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/common.h"

#include "pcl/filters/extract_indices.h"
#include <string>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud,
                  pcl::PointIndices::Ptr indices,
                  pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  seg.segment(indices_internal, *coeff);

  // Build custom indices that ignores points above the plane.
  for (size_t i=0; i<indices_internal.indices.size(); ++i) {
    int index = indices_internal.indices[i];
    const PointC& pt = cloud->points[index];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(index);
    }
  }

  // coeff_out = boost::make_shared<pcl::ModelCoefficients>(coeff);

  // pcl::ModelCoefficients::Ptr coeff_tmp = pcl::ModelCoefficients::Ptr(&coeff);
  // coeff_out.swap(coeff_tmp);

  // coeff_out->values.push_back(coeff.values[0]);
  // coeff_out->values.push_back(coeff.values[1]);
  // coeff_out->values.push_back(coeff.values[2]);
  // coeff_out->values.push_back(coeff.values[3]);
  
  // coeff_out.reset(&coeff);

  // *indices = indices_internal;

  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }  
}

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {
  pcl::ExtractIndices<PointC> extract;
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  ROS_INFO("1_cloud: %ld", cloud->size());

  ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*object_indices);

  // Find the size of the smallest and the largest object,
  // where size = number of points in the cluster
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < object_indices->size(); ++i) {
    // TODO: implement this
    size_t cluster_size = (*object_indices)[i].indices.size();
    if (cluster_size > max_size) max_size = cluster_size;
    if (cluster_size < min_size) min_size = cluster_size;
  }

  ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
          object_indices->size(), min_size, max_size);
}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
  // ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);
  pose->position.x = (min_pcl.x + max_pcl.x) / 2;
  pose->position.y = (min_pcl.y + max_pcl.y) / 2;
  pose->position.z = (min_pcl.z + max_pcl.z) / 2;
  dimensions -> x = max_pcl.x - min_pcl.x;
  dimensions -> y = max_pcl.y - min_pcl.y; 
  dimensions -> z = max_pcl.z - min_pcl.z;
  pose->orientation.w = 1;

}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
const ros::Publisher& marker_pub)
    : surface_points_pub_(surface_points_pub),
    marker_pub_(marker_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  SegmentSurface(cloud, table_inliers, coeff);

  // Extract subset of original_cloud into subset_cloud:
  PointCloudC::Ptr subset_cloud(new PointCloudC());
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*subset_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  surface_points_pub_.publish(msg_out); 

  ROS_INFO("Table_cloud: %ld", cloud->size());
  ROS_INFO("Subset_cloud: %ld", subset_cloud->size());

  PointCloudC::Ptr extract_out(new PointCloudC());
  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose table_pose;
  simple_grasping::extractShape(*subset_cloud, coeff, *extract_out, shape, table_pose);

  // Sets the pose and dimensions of a box surrounding the given point cloud.
  visualization_msgs::Marker table_marker;
  table_marker.ns = "table";
  table_marker.header.frame_id = "base_link";
  table_marker.type = visualization_msgs::Marker::CUBE;
  table_marker.pose = table_pose;
  if (shape.type == shape_msgs::SolidPrimitive::BOX) {
    table_marker.scale.x = shape.dimensions[0];
    table_marker.scale.y = shape.dimensions[1];
    table_marker.scale.z = shape.dimensions[2];
  } else {
    ROS_INFO("TABLE SHAPE IS NOT A BOX, LOL");
  }
  // GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
  table_marker.color.r = 1;
  table_marker.color.a = 0.8;
  marker_pub_.publish(table_marker);

  PointCloudC::Ptr cloud2(new PointCloudC());
  //pcl::fromROSMsg(msg, *cloud2);
  std::vector<pcl::PointIndices> object_indices;
  pcl::PointIndices::Ptr table_inliers2(new pcl::PointIndices());
  SegmentSurfaceObjects(cloud, table_inliers, &object_indices);
  ROS_INFO("hello");

  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    pcl::ExtractIndices<PointC> extract2;
    extract2.setInputCloud(cloud);
    extract2.setIndices(indices);
    extract2.filter(*object_cloud);

    PointCloudC::Ptr extract_out2(new PointCloudC());
    shape_msgs::SolidPrimitive shape2;
    geometry_msgs::Pose table_pose2;
    simple_grasping::extractShape(*object_cloud, *extract_out2, shape2, table_pose2);

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.pose = table_pose2;
    if (shape2.type == shape_msgs::SolidPrimitive::BOX) {
      object_marker.scale.x = shape2.dimensions[0];
      object_marker.scale.y = shape2.dimensions[1];
      object_marker.scale.z = shape2.dimensions[2];
    } else if (shape2.type == shape_msgs::SolidPrimitive::SPHERE) {
      ROS_INFO("Some object is sphere");
    } else if (shape2.type == shape_msgs::SolidPrimitive::CYLINDER) {
      ROS_INFO("Some object is cylinder");
    } else if (shape2.type == shape_msgs::SolidPrimitive::CONE) {
      ROS_INFO("Some object is cone");
    } else {
      ROS_INFO("Some object is ???");
    }
    // GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
    //                           &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub_.publish(object_marker);
  }

  //SegmentSurface(cloud, table_inliers2);
  // for (size_t i = 0; i < table_inliers->indices.size(); ++i) {
  //   ROS_INFO("%d" + table_inliers->indices[i]);
  // }

  // We are reusing the extract object created earlier in the callback.
  // PointCloudC::Ptr cloud_out(new PointCloudC());
  // extract.setNegative(true);
  // extract.filter(*cloud_out);
  // pcl::toROSMsg(*cloud_out, msg_out);
  // marker_pub.publish(msg_out);
  //above_surface_pub_.publish(msg_out);
}
}  // namespace perception
