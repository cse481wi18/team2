#include "perception/segmentation.h"
#include "perception/box_fitter.h"
#include "perception/object.h"
#include "perception/object_recognizer.h"

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

#include "perception_msgs/TennisBallPoses.h"

#include <math.h>
#include <sstream>
#include <string>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {



  void SegmentTopScene(PointCloudC::Ptr above_cloud,
                            std::vector<Object>* objects) {

    // sensor_msgs::PointCloud2 test_out;
    // pcl::toROSMsg(*above_cloud, test_out);
    // ros::NodeHandle nh;
    // ros::Publisher test_pub = nh.advertise<sensor_msgs::PointCloud2>("test_cloud", 1, true);
    // test_pub.publish(test_out); 

    // Same as callback, but with visualization code removed.
    std::vector<pcl::PointIndices> object_indices;
    SegmentSurfaceObjects(above_cloud, &object_indices);

    // std::cout << "hello" << std::endl;
    for (size_t i = 0; i < object_indices.size(); ++i) {
      // Reify indices into a point cloud of the object.
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      *indices = object_indices[i];
      PointCloudC::Ptr object_cloud(new PointCloudC());
      // TODO: fill in object_cloud using indices
      pcl::ExtractIndices<PointC> extract2; 
      extract2.setInputCloud(above_cloud);
      extract2.setIndices(indices);
      extract2.filter(*object_cloud);

      PointCloudC::Ptr extract_out2(new PointCloudC());
      shape_msgs::SolidPrimitive shape2;
      geometry_msgs::Pose table_pose2;

      pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
      double table_coeff_0;
      ros::param::param("table_coeff_0", table_coeff_0, 0.0);
      double table_coeff_1;
      ros::param::param("table_coeff_1", table_coeff_1, 0.0);
      double table_coeff_2;
      ros::param::param("table_coeff_2", table_coeff_2, 1.0);
      double table_coeff_3;
      ros::param::param("table_coeff_3", table_coeff_3, 0.0);
      coeff->values.push_back(table_coeff_0);
      coeff->values.push_back(table_coeff_1);
      coeff->values.push_back(table_coeff_2);
      coeff->values.push_back(table_coeff_3);
      FitBox(*object_cloud, coeff, *extract_out2, shape2, table_pose2);

      Object obj = Object();
      obj.name = "some object";
      // obj.confidence = ???;
      obj.cloud = object_cloud;
      obj.pose = table_pose2;
      obj.dimensions.x = shape2.dimensions[0];
      obj.dimensions.y = shape2.dimensions[1];
      obj.dimensions.z = shape2.dimensions[2];
      objects->push_back(obj);
    }
  }

  void SegmentTabletopScene(PointCloudC::Ptr table_and_above_cloud,
                            std::vector<Object>* objects) {
    ros::NodeHandle nh;
    ros::Publisher table_pub =
        nh.advertise<sensor_msgs::PointCloud2>("test_table_cloud", 1, true);
    ros::Publisher above_pub =
        nh.advertise<sensor_msgs::PointCloud2>("test_above_cloud", 1, true);


    // Separate table and above
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    SegmentSurface(table_and_above_cloud, table_inliers, coeff);
    if (table_inliers->indices.size() == 0) {
      ROS_INFO("Can't find table! Aborting...");
      return;
    }

    // Get above_cloud
    PointCloudC::Ptr above_cloud(new PointCloudC());
    pcl::ExtractIndices<PointC> extract2;
    extract2.setInputCloud(table_and_above_cloud);
    extract2.setNegative(true);
    extract2.setIndices(table_inliers);
    extract2.filter(*above_cloud);

    SegmentTopScene(above_cloud, objects);
  }

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
    double surface_threshold;
    ros::param::param("surface_threshold", surface_threshold, 0.01);
    seg.setDistanceThreshold(surface_threshold);
    seg.setInputCloud(cloud);

    double surface_angle_threshold;
    ros::param::param("surface_angle_threshold", surface_angle_threshold, 10.0);

    // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(surface_angle_threshold));

    std::cout << "surface_threshold = " << surface_threshold << std::endl;
    std::cout << "surface_angle_threshold = " << surface_angle_threshold << std::endl;

    // coeff contains the coefficients of the plane:
    // ax + by + cz + d = 0
    double distance_above_plane;
    ros::param::param("distance_above_plane", distance_above_plane, 0.005);
    std::cout << "distance_above_plane = " << distance_above_plane << std::endl;

    seg.segment(indices_internal, *coeff);
    
    // Print coefficients    // for (std::vector<float>::const_iterator i = coeff->values.begin(); i != coeff->values.end(); ++i)
    //     std::cout << *i << std::endl;
    for (std::vector<float>::const_iterator i = coeff->values.begin(); i != coeff->values.end(); ++i)
        std::cout << *i << std::endl;

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

    if (indices->indices.size() == 0) {
      ROS_ERROR("Unable to find surface.");
      return;
    }  
  }

  void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr above_surface_cloud,
                            std::vector<pcl::PointIndices>* object_indices) {
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 20);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 1000);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(above_surface_cloud);
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

  Segmenter::Segmenter(const ros::Publisher& table_pub,
  const ros::Publisher& marker_pub,
  const ros::Publisher& above_surface_pub,
  const ros::Publisher& ball_poses_pub,
  const ObjectRecognizer& recognizer)
      : table_pub_(table_pub),
      marker_pub_(marker_pub),
      above_surface_pub_(above_surface_pub),
      ball_poses_pub_(ball_poses_pub),
      recognizer_(recognizer) {}

  void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    // to ROS msg
    PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud_unfiltered);

    // remove NaN points
    PointCloudC::Ptr cloud(new PointCloudC());
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

    std::cout << "Cloud has " << cloud->points.size() << " indices" << std::endl;

    //************ REDUNDANT CODE FOR DEBUG PURPOSES - DO NOT REMOVE ***************
    // get the objects on the table top
    std::vector<Object> objects;

    // Separate table and above
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    SegmentSurface(cloud, table_inliers, coeff);
    if (table_inliers->indices.size() == 0) {
      ROS_INFO("Can't find table! Aborting...");
      return;
    } else {
      std::cout << "Table has " << table_inliers->indices.size() << " indices" << std::endl;
    }

    ros::NodeHandle nh;
    nh.setParam("table_coeff_0", coeff->values[0]);
    nh.setParam("table_coeff_1", coeff->values[1]);
    nh.setParam("table_coeff_2", coeff->values[2]);
    nh.setParam("table_coeff_3", coeff->values[3]);

    // Get above_cloud
    PointCloudC::Ptr table_cloud(new PointCloudC());
    pcl::ExtractIndices<PointC> extract3;
    extract3.setInputCloud(cloud);
    extract3.setIndices(table_inliers);
    extract3.filter(*table_cloud);

    // Get above_cloud
    PointCloudC::Ptr above_cloud(new PointCloudC());
    pcl::ExtractIndices<PointC> extract2;
    extract2.setInputCloud(cloud);
    extract2.setNegative(true);
    extract2.setIndices(table_inliers);
    extract2.filter(*above_cloud);

    SegmentTopScene(above_cloud, &objects);

    table_pub_.publish(table_cloud);
    above_surface_pub_.publish(above_cloud);
    //************* REDUNDANT CODE FOR DEBUG PURPOSES - DO NOT REMOVE ****************

    perception_msgs::TennisBallPoses tennis_ball_poses2;

    std::cout << "Found " << objects.size() << " objects" << std::endl;

    // make a bounding box around each objects
    for (size_t i = 0; i < objects.size(); ++i) {
      const Object& object = objects[i];
      
      // Recognize the object.
      std::string name;
      double confidence;
      //recognizer_.Recognize(object, &name, &confidence);
      confidence = recognizer_.RecognizeIndex(object, 0);
      confidence = round(1000 * confidence) / 1000;

      double recognize_threshold;
      ros::param::param("recognize_threshold", recognize_threshold, 1000.0);

      if (confidence < recognize_threshold) {
        tennis_ball_poses2.poses.push_back(object.pose);
      }
      ball_poses_pub_.publish(tennis_ball_poses2);
      
      // Publish a bounding box around it.
      visualization_msgs::Marker object_marker;
      object_marker.ns = "objects";
      object_marker.id = i;
      object_marker.header.frame_id = "base_link";
      object_marker.type = visualization_msgs::Marker::CUBE;
      object_marker.pose = object.pose;
      object_marker.scale = object.dimensions;
      object_marker.color.g = 1;
      object_marker.color.a = 0.3;
      if (confidence < recognize_threshold) {
        object_marker.color.b = 1;
      }

      marker_pub_.publish(object_marker);


      std::stringstream ss;
      ss << name << " (" << confidence << ")";

      // Publish the recognition result.
      visualization_msgs::Marker name_marker;
      name_marker.ns = "recognition";
      name_marker.id = i;
      name_marker.header.frame_id = "base_link";
      name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      name_marker.pose.position = object.pose.position;
      name_marker.pose.position.z += 0.1;
      name_marker.pose.orientation.w = 1;
      name_marker.scale.x = 0.025;
      name_marker.scale.y = 0.025;
      name_marker.scale.z = 0.025;
      name_marker.color.r = 0;
      name_marker.color.g = 0;
      name_marker.color.b = 1.0;
      name_marker.color.a = 1.0;
      name_marker.text = ss.str();

      marker_pub_.publish(name_marker);
    }

    // for (std::vector<geometry_msgs::Pose>::const_iterator i = tennis_ball_poses2.poses.begin(); i != tennis_ball_poses2.poses.end(); ++i)
    //   std::cout << *i;
  }
} // namespace perception

// TABLE MARKER PUBLISHER
//   // Publish table marker
//   PointCloudC::Ptr extract_out(new PointCloudC());
//   shape_msgs::SolidPrimitive shape;
//   geometry_msgs::Pose table_pose;
//   simple_grasping::extractShape(*subset_cloud, coeff, *extract_out, shape, table_pose);

// // Sets the pose and dimensions of a box surrounding the given point cloud.
// visualization_msgs::Marker table_marker;
// table_marker.ns = "table";
// table_marker.header.frame_id = "base_link";
// table_marker.type = visualization_msgs::Marker::CUBE;
// table_marker.pose = table_pose;
// if (shape.type == shape_msgs::SolidPrimitive::BOX) {
//   table_marker.scale.x = shape.dimensions[0];
//   table_marker.scale.y = shape.dimensions[1];
//   table_marker.scale.z = shape.dimensions[2];
// } else {
//   ROS_INFO("TABLE SHAPE IS NOT A BOX, LOL");
// }
// table_marker.color.r = 1;
// table_marker.color.a = 0.8;
// marker_pub_.publish(table_marker);
// // Done publishing table marker
