/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann, Otniel Rinaldo
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <costmap_converter/costmap_converter_interface.h>

class CostmapStandaloneConversion : public rclcpp::Node 
{
 public:
  explicit CostmapStandaloneConversion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void healthCheck();
  void publishCallback();
  void publishAsMarker(
      const std::string &frame_id,
      const std::vector<geometry_msgs::msg::PolygonStamped> &polygonStamped);
  void publishAsMarker(
      const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles);

 private:
  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons>
      converter_loader_;
  std::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;

  rclcpp::Node::SharedPtr n_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<std::thread> costmap_thread_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr
      obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;

  std::string frame_id_;
  int occupied_min_value_;
  int conversion_interval_;
  rclcpp::Time last_publish_time_;
  bool respawn_ = false;
  rclcpp::CallbackGroup::SharedPtr cb_group1_;
  rclcpp::CallbackGroup::SharedPtr cb_group2_;

};
