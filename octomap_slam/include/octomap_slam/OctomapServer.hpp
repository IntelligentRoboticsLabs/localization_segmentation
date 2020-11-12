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

#ifndef OCTOMAP_SLAM__OCTOMAP_SERVER_HPP_
#define OCTOMAP_SLAM__OCTOMAP_SERVER_HPP_

#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"
#include "octomap/ColorOcTree.h"

#include "tf2_ros/transform_listener.h"
#include "tf2/buffer_core.h"

#include "octomap_slam_msgs/srv/save_map.hpp"
#include "octomap_msgs/msg/octomap.hpp"

#include "rclcpp/rclcpp.hpp"

namespace octomap_server
{

class OctomapServer : public rclcpp::Node
{
public:
  OctomapServer();

protected:
  void save_map_callback(
    const std::shared_ptr<octomap_slam_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<octomap_slam_msgs::srv::SaveMap::Response> response);

  void octomap_perceptions_callback(octomap_msgs::msg::Octomap::UniquePtr msg);

private:
  rclcpp::Service<octomap_slam_msgs::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_perceptions_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback();

  octomap::ColorOcTree * octomap_;

  std::shared_ptr<tf2::BufferCore> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double voxel_res_;
  bool mapping_;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SLAM__OCTOMAP_SERVER_HPP_
