// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tf2_ros/buffer_interface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "octomap_slam/OctomapServer.hpp"
#include "octomap_msgs/conversions.h"

namespace octomap_server
{

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

OctomapServer::OctomapServer()
: Node("octomap_server"), localization_quality_(1.0)
{
  save_map_service_ = create_service<octomap_slam_msgs::srv::SaveMap>(
    "~/save_map", std::bind(&OctomapServer::save_map_callback, this, _1, _2));

  octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
    "~/octomap", rclcpp::QoS(1).transient_local());  
    
  timer_ = create_wall_timer(
    1s, std::bind(&OctomapServer::timer_callback, this));

  this->declare_parameter("voxel_resolution", 0.1);
  this->declare_parameter("octomap_file", std::string(""));
  this->declare_parameter("mapping", true);
  
  this->get_parameter("voxel_resolution", voxel_res_);
  this->get_parameter("mapping", mapping_);

  if (mapping_) {
    octomap_perceptions_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "~/input_octomaps", 100,
      std::bind(&OctomapServer::octomap_perceptions_callback, this, _1));
    localization_info_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 100,
      std::bind(&OctomapServer::localization_info_callback, this, _1));  }

  tfBuffer_ = std::make_shared<tf2::BufferCore>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this, false);

  std::string octomap_file;
  this->get_parameter("octomap_file", octomap_file);

  if (octomap_file != "") {
    octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(octomap_file);
    octomap_ = dynamic_cast<octomap::ColorOcTree*>(tree);

    if (octomap_ != NULL) {
      RCLCPP_INFO(get_logger(), "Octomap loaded [%s]", octomap_file);
    } else {
      RCLCPP_ERROR(get_logger(), "Error loading octomap [%s]", octomap_file);
    }
  } else {
    double probHit, probMiss, thresMin, thresMax;
    probHit = 0.7;
    probMiss = 0.4;
    thresMin = 0.12;
    thresMax = 0.97;
  
    octomap_ = new octomap::ColorOcTree(voxel_res_);
    octomap_->setProbHit(probHit);
    octomap_->setProbMiss(probMiss);
    octomap_->setClampingThresMin(thresMin);
    octomap_->setClampingThresMax(thresMax);
  }
}

void
OctomapServer::save_map_callback(
  const std::shared_ptr<octomap_slam_msgs::srv::SaveMap::Request> request,
  std::shared_ptr<octomap_slam_msgs::srv::SaveMap::Response>      response)
{
  response->success = octomap_->write(request->path);


}

void
OctomapServer::localization_info_callback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg)
{
  localization_quality_ = (1.0 - sqrt(msg->pose.covariance[0])) * (1.0 - sqrt(msg->pose.covariance[7])); // x^2 and y^2
  std::cerr << "==> " << localization_quality_ << std::endl;
}

void
OctomapServer::octomap_perceptions_callback(octomap_msgs::msg::Octomap::UniquePtr msg)
{
  std::string error;
  
  if (tfBuffer_->canTransform("map", msg->header.frame_id,
      tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()), &error))
  {
    try {
      geometry_msgs::msg::TransformStamped tf;
      auto tf2map_msg = tfBuffer_->lookupTransform("map", msg->header.frame_id,
        tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()));

      tf2::Stamped<tf2::Transform> tf2map;
      tf2::fromMsg(tf2map_msg, tf2map);
    
      octomap::ColorOcTree* received_octree = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::msgToMap(*msg));

      for (auto it = received_octree->begin_leafs(); it != received_octree->end_leafs(); ++it) {
        if (received_octree->isNodeOccupied(*it)) {
          tf2::Vector3 p_in(it.getX(), it.getY(), it.getZ());
          tf2::Vector3 p_map = tf2map * p_in;

          octomap_->updateNode(p_map.x(), p_map.y(), p_map.z(), false);

          double new_prob = localization_quality_;          
          double previous_prob;

          auto node = octomap_->search(p_map.x(), p_map.y(), p_map.z());
          if (node != nullptr) {

            if (static_cast<double>(node->getValue() < 0)) {
              previous_prob = 0.1;
            } else {
              previous_prob = static_cast<double>(node->getValue());
            }
            
            new_prob = (previous_prob + it->getValue() * localization_quality_) / 2.0;
          }
          std::cerr << previous_prob << "(" << node->getValue() << ") + " << localization_quality_ << " * " <<  it->getValue() << " / 2 = " << new_prob << std::endl;
          octomap_->setNodeValue(p_map.x(), p_map.y(), p_map.z(), new_prob, true);
          octomap_->setNodeColor(p_map.x(), p_map.y(), p_map.z(), it->getColor().r, it->getColor().g, it->getColor().b);
        }
      }
      
      std::cerr << std::endl;

      octomap_msgs::msg::Octomap octomap_msg;
      octomap_msg.header.frame_id = "map";
      octomap_msg.header.stamp = msg->header.stamp;
      octomap_msg.binary = false;
      octomap_msg.resolution = voxel_res_;
      
      size_t octomapSize = octomap_->size();
      if (octomapSize < 1){
        RCLCPP_WARN(get_logger(),"Nothing to publish, octree is empty");
        return;
      }
    
      if (octomap_msgs::fullMapToMsg(*octomap_, octomap_msg)){
        octomap_pub_->publish(octomap_msg);
      }else{
        RCLCPP_ERROR(get_logger(),"Error serializing OctoMap");
      }

      delete received_octree;
    } catch (std::exception & e) {
      RCLCPP_WARN(get_logger(), "octomap_perceptions_callback: %s", e.what());
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Error in octomap callback: %s", error.c_str());
  }
}

void 
OctomapServer::timer_callback()
{
   octomap_msgs::msg::Octomap octomap_msg;
   octomap_msg.header.frame_id = "map";
   octomap_msg.header.stamp = now();
   octomap_msg.binary = false;
   octomap_msg.resolution = voxel_res_;
   
   size_t octomapSize = octomap_->size();
   if (octomapSize < 1){
     RCLCPP_WARN(get_logger(),"Nothing to publish, octree is empty");
     return;
   }
 
   if (octomap_msgs::fullMapToMsg(*octomap_, octomap_msg)){
     octomap_pub_->publish(octomap_msg);
   }else{
     RCLCPP_ERROR(get_logger(),"Error serializing OctoMap");
   }
}


}  // namespace octomap_server
