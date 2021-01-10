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

#include <random>
#include <algorithm>

#include "tf2_ros/buffer_interface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

#include "octomap_slam/OctomapServer.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap_utils.h"

namespace octomap_server
{

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

OctomapServer::OctomapServer()
: Node("octomap_server")
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
  pose_with_cov_ = std::move(msg);
}

void
OctomapServer::octomap_perceptions_callback(octomap_msgs::msg::Octomap::UniquePtr msg)
{
  std::string error;
  // octomap_->clear();

  if (pose_with_cov_ == nullptr) {
    std::cerr << "Not pose with covariance received yet" << std::endl;
  }

  if (tfBuffer_->canTransform("map", msg->header.frame_id,
      tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()), &error))
  {
    try {

      auto map2bf_msg = tfBuffer_->lookupTransform("map", "base_footprint",
        tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()));
      auto bf2tf_msg = tfBuffer_->lookupTransform("base_footprint", msg->header.frame_id,
        tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()));

      tf2::Stamped<tf2::Transform> map2bf, bf2tf;
      tf2::fromMsg(map2bf_msg, map2bf);
      tf2::fromMsg(bf2tf_msg, bf2tf);

      octomap::ColorOcTree* received_octree = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::msgToMap(*msg));

      int samples = 1;
      std::default_random_engine generator;
      std::normal_distribution<double> dist_x(0.0, (pose_with_cov_->pose.covariance[0]));
      std::normal_distribution<double> dist_y(0.0, (pose_with_cov_->pose.covariance[7]));
      std::normal_distribution<double> dist_z(0.0, (pose_with_cov_->pose.covariance[14]));
      std::normal_distribution<double> dist_roll(0.0, (pose_with_cov_->pose.covariance[21]));
      std::normal_distribution<double> dist_pitch(0.0, (pose_with_cov_->pose.covariance[28]));
      std::normal_distribution<double> dist_yaw(0.0, (pose_with_cov_->pose.covariance[35]));

      double roll, pitch, yaw;
      map2bf.getRotation().normalized().setEuler(yaw, pitch, roll);

      double K = 0.9;

      for (int i = 0; i < samples; i++) {
        tf2::Transform noise;
        noise.setOrigin(tf2::Vector3(dist_x(generator), dist_y(generator), dist_z(generator)));
        
        tf2::Quaternion q;
        q.setRPY(dist_roll(generator), dist_pitch(generator), dist_yaw(generator));
        noise.setRotation(q);

        for (auto it = received_octree->begin_leafs(); it != received_octree->end_leafs(); ++it) {
          if (received_octree->isNodeOccupied(*it)) {
            tf2::Vector3 p_in(it.getX(), it.getY(), it.getZ());
            tf2::Vector3 p_map =  map2bf * noise * bf2tf * p_in;
        
            double new_prob;          
  
            octomap::OcTreeKey key = octomap_->coordToKey(octomap::point3d(p_map.x(), p_map.y(), p_map.z()));
            auto node = octomap_->search(key);

            if (node != nullptr) {
              double previous_prob = std::clamp(static_cast<double>(node->getOccupancy()), 0.0, 1.0);
              new_prob = std::clamp(K * previous_prob + (1.0 - K) * it->getOccupancy() / static_cast<double>(samples), 0.0, 1.0);
              // std::cerr << new_prob << "=" << K << " * " << previous_prob << "+ (1.0 - " << K << ") * " << it->getOccupancy() << " / " << static_cast<double>(samples) << std::endl;;
            } else {
              node = octomap_->updateNode(p_map.x(), p_map.y(), p_map.z(), true);
              new_prob = 0.1; // it->getOccupancy() / static_cast<double>(samples);
            }
           
            
            node->setLogOdds(octomap::logodds(new_prob));
            node->setColor(it->getColor().r, it->getColor().g, it->getColor().b);
          }
        }
      }

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
