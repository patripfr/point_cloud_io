/*
 * Write.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Write.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace point_cloud_io {

Write::Write(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), filePrefix_("point_cloud"), fileEnding_("ply") {
  if (!readParameters()) {
    ros::requestShutdown();
  }
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &Write::pointCloudCallback, this);
  service_ = nodeHandle_.advertiseService("save_ply", &Write::saveCallback, this);
  ROS_INFO_STREAM("Subscribed to topic \"" << pointCloudTopic_ << "\".");
}

bool Write::readParameters() {
  bool allParametersRead = true;
  allParametersRead = nodeHandle_.getParam("topic", pointCloudTopic_) && allParametersRead;
  allParametersRead = nodeHandle_.getParam("folder_path", folderPath_) && allParametersRead;

  nodeHandle_.getParam("file_prefix", filePrefix_);
  nodeHandle_.getParam("file_ending", fileEnding_);
  nodeHandle_.getParam("add_counter_to_path", addCounterToPath_);
  nodeHandle_.getParam("add_frame_id_to_path", addFrameIdToPath_);
  nodeHandle_.getParam("add_stamp_sec_to_path", addStampSecToPath_);
  nodeHandle_.getParam("add_stamp_nsec_to_path", addStampNSecToPath_);
  nodeHandle_.getParam("accumulate", accumulate_);

  if (!allParametersRead) {
    ROS_WARN(
        "Could not read all parameters. Typical command-line usage:\n"
        "rosrun point_cloud_io write"
        " _topic:=/my_topic"
        " _folder_path:=/home/user/my_point_clouds"
        " (optional: _file_prefix:=my_prefix"
        " _file_ending:=my_ending"
        " _add_counter_to_path:=true/false"
        " _add_frame_id_to_path:=true/false"
        " _add_stamp_sec_to_path:=true/false"
        " _add_stamp_nsec_to_path:=true/false)");
    return false;
  }

  return true;
}

bool Write::saveCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  std::cout << folderPath_ << std::endl;
  std::stringstream filePath;
  filePath << folderPath_ << "/";
  if (!filePrefix_.empty()) {
    filePath << filePrefix_;
  }
  if (addCounterToPath_) {
    filePath << "_" << counter_;
    counter_++;
  }

  filePath << ".";
  filePath << fileEnding_;
  if (fileEnding_ == "ply") {
    pcl::PLYWriter writer;
    bool binary = false;
    bool use_camera = false;
    if (writer.write(filePath.str(), accumulated_cloud_, binary, use_camera) != 0) {
      ROS_ERROR("Something went wrong when trying to write the point cloud file.");
      return false;
    }
  } else if (fileEnding_ == "pcd") {
    // Write pcd file
    pcl::io::savePCDFile(filePath.str(), accumulated_cloud_);
  } else {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }
    return true;
}

void Write::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  ROS_INFO_STREAM("Received point cloud with " << cloud->height * cloud->width << " points.");
  std::cout << folderPath_ << std::endl;
  std::stringstream filePath;
  filePath << folderPath_ << "/";
  if (!filePrefix_.empty()) {
    filePath << filePrefix_;
  }
  if (addCounterToPath_) {
    filePath << "_" << counter_;
    counter_++;
  }
  if (addFrameIdToPath_) {
    filePath << "_" << cloud->header.frame_id;
  }
  if (addStampSecToPath_) {
    filePath << "_" << cloud->header.stamp.sec;
  }
  if (addStampNSecToPath_) {
    filePath << "_" << cloud->header.stamp.nsec;
  }
  filePath << ".";
  filePath << fileEnding_;

  if (accumulate_) {
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*cloud, pclCloud);
    accumulated_cloud_ += pclCloud;
    ROS_INFO_STREAM("Accumulated point cloud with " << accumulated_cloud_.height * accumulated_cloud_.width << " points.");
    return;
  }

  if (fileEnding_ == "ply") {


    // Write .ply file.
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*cloud, pclCloud);

    pcl::PLYWriter writer;
    bool binary = false;
    bool use_camera = false;
    if (writer.write(filePath.str(), pclCloud, binary, use_camera) != 0) {
      ROS_ERROR("Something went wrong when trying to write the point cloud file.");
      return;
    }
  } else if (fileEnding_ == "pcd") {
    // Write pcd file
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*cloud, pclCloud);
    pcl::io::savePCDFile(filePath.str(), pclCloud);
  } else {
    ROS_ERROR_STREAM("Data format not supported.");
    return;
  }

  ROS_INFO_STREAM("Saved point cloud to " << filePath.str() << ".");
}

}  // namespace point_cloud_io
