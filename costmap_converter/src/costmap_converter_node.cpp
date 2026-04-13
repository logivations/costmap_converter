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
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_converter/costmap_converter_node.h>
#include <pluginlib/class_loader.hpp>

CostmapStandaloneConversion::CostmapStandaloneConversion(const rclcpp::NodeOptions & options)
    : rclcpp::Node("costmap_converter", options),
      converter_loader_("costmap_converter",
                        "costmap_converter::BaseCostmapToPolygons") {
  n_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  std::string converter_plugin =
      "costmap_converter::CostmapToPolygonsDBSMCCH";
  declare_parameter("converter_plugin",
                    rclcpp::ParameterValue(converter_plugin));
  get_parameter_or<std::string>("converter_plugin", converter_plugin,
                                converter_plugin);

  try {
    converter_ = converter_loader_.createSharedInstance(converter_plugin);
  } catch (const pluginlib::PluginlibException &ex) {
    RCLCPP_ERROR(get_logger(),
                  "The plugin failed to load for some reason. Error: %s",
                  ex.what());
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "Standalone costmap converter: %s loaded.",
              converter_plugin.c_str());

  std::string obstacles_topic = "costmap_obstacles";
  declare_parameter("obstacles_topic",
                    rclcpp::ParameterValue(obstacles_topic));
  get_parameter_or<std::string>("obstacles_topic", obstacles_topic,
                                obstacles_topic);

  std::string polygon_marker_topic = "costmap_polygon_markers";
  declare_parameter("polygon_marker_topic",
                    rclcpp::ParameterValue(polygon_marker_topic));
  get_parameter_or<std::string>("polygon_marker_topic", polygon_marker_topic,
                                polygon_marker_topic);

  obstacle_pub_ =
      create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(
          obstacles_topic, 1000);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      polygon_marker_topic, 10);

  occupied_min_value_ = 100;
  declare_parameter("occupied_min_value",
                    rclcpp::ParameterValue(occupied_min_value_));
  get_parameter_or<int>("occupied_min_value", occupied_min_value_,
                        occupied_min_value_);

  conversion_interval_ = 0;
  declare_parameter("conversion_interval",
                    rclcpp::ParameterValue(conversion_interval_));
  get_parameter_or<int>("conversion_interval", conversion_interval_,
                        conversion_interval_);

  std::string odom_topic = "/odom";
  declare_parameter("odom_topic", rclcpp::ParameterValue(odom_topic));
  get_parameter_or<std::string>("odom_topic", odom_topic, odom_topic);

  std::string global_plan_topic = "/plan";
  declare_parameter("global_plan_topic", rclcpp::ParameterValue(global_plan_topic));
  get_parameter_or<std::string>("global_plan_topic", global_plan_topic, global_plan_topic);

  // TF buffer for global plan transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  global_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_plan_topic, 1,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (!converter_)
          return;
        if (msg->header.frame_id.empty() || msg->header.frame_id == frame_id_) {
          converter_->setGlobalPlan(msg->poses);
          return;
        }
        // Transform plan poses from plan frame into costmap frame
        try {
          geometry_msgs::msg::TransformStamped transform =
              tf_buffer_->lookupTransform(frame_id_, msg->header.frame_id, tf2::TimePointZero);
          std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
          transformed_plan.reserve(msg->poses.size());
          for (const auto& pose : msg->poses) {
            geometry_msgs::msg::PoseStamped transformed_pose;
            tf2::doTransform(pose, transformed_pose, transform);
            transformed_plan.push_back(transformed_pose);
          }
          converter_->setGlobalPlan(transformed_plan);
        } catch (const tf2::TransformException& ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Could not transform global plan from '%s' to '%s': %s",
              msg->header.frame_id.c_str(), frame_id_.c_str(), ex.what());
          converter_->setGlobalPlan(msg->poses);
        }
      });

  if (converter_) {
    converter_->setOdomTopic(odom_topic);
    converter_->initialize(shared_from_this());
  }

  cb_group1_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);


  if (conversion_interval_ > 0) {
    // Timer-based mode: use full Costmap2DROS
    costmap_ros_ =
        std::make_shared<nav2_costmap_2d::Costmap2DROS>("converter_costmap",
           std::string{get_namespace()}, get_parameter("use_sim_time").as_bool());
    costmap_thread_ = std::make_unique<std::thread>(
        [](rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
          rclcpp::spin(node->get_node_base_interface());
        },
        costmap_ros_);
    rclcpp_lifecycle::State state;
    costmap_ros_->on_configure(state);
    costmap_ros_->on_activate(state);

    if (converter_) {
      converter_->setCostmap2D(costmap_ros_->getCostmap());
    }

    pub_timer_ = n_->create_wall_timer(
        std::chrono::milliseconds(conversion_interval_),
        std::bind(&CostmapStandaloneConversion::publishCallback, this), cb_group1_);
  } else {
    // Event-driven mode: own a lightweight Costmap2D, subscribe to OccupancyGrid
    costmap_direct_ = std::make_shared<nav2_costmap_2d::Costmap2D>();

    if (converter_) {
      converter_->setCostmap2D(costmap_direct_.get());
    }

    std::string costmap_topic = "costmap";
    declare_parameter("costmap_topic",
                      rclcpp::ParameterValue(costmap_topic));
    get_parameter_or<std::string>("costmap_topic", costmap_topic,
                                  costmap_topic);

    RCLCPP_INFO(get_logger(),
        "Event-driven mode: converting on each costmap update from '%s'",
        costmap_topic.c_str());

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group1_;
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic, rclcpp::SensorDataQoS(),
        std::bind(&CostmapStandaloneConversion::costmapCallback, this,
                  std::placeholders::_1),
        sub_opts);
  }

}


void CostmapStandaloneConversion::costmapCallback(
    nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
  frame_id_ = msg->header.frame_id;

  unsigned int size_x = msg->info.width;
  unsigned int size_y = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  {
    auto* mutex = costmap_direct_->getMutex();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

    costmap_direct_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    unsigned char* costmap_data = costmap_direct_->getCharMap();
    for (unsigned int i = 0; i < size_x * size_y; ++i) {
      int8_t value = msg->data[i];
      if (value < 0) {
        costmap_data[i] = nav2_costmap_2d::NO_INFORMATION;
      } else if (value >= occupied_min_value_) {
        costmap_data[i] = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else {
        costmap_data[i] = nav2_costmap_2d::FREE_SPACE;
      }
    }
  }

  publishCallback();
}

void CostmapStandaloneConversion::publishCallback() {
  converter_->workerCallback();
  costmap_converter::ObstacleArrayPtr obstacles =
      converter_->getObstacles();
  if (!obstacles) return;
  if (costmap_ros_) {
    frame_id_ = costmap_ros_->getGlobalFrameID();
  }
  obstacles->header.frame_id = frame_id_;
  obstacles->header.stamp = now();
  obstacle_pub_->publish(*obstacles);
  publishAsMarker(*obstacles);
}

void CostmapStandaloneConversion::publishAsMarker(
    const std::string &frame_id,
    const std::vector<geometry_msgs::msg::PolygonStamped> &polygonStamped) {
  visualization_msgs::msg::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = now();
  line_list.ns = "Polygons";
  line_list.action = visualization_msgs::msg::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;

  for (std::size_t i = 0; i < polygonStamped.size(); ++i) {
    for (int j = 0; j < (int)polygonStamped[i].polygon.points.size() - 1;
          ++j) {
      geometry_msgs::msg::Point line_start;
      line_start.x = polygonStamped[i].polygon.points[j].x;
      line_start.y = polygonStamped[i].polygon.points[j].y;
      line_list.points.push_back(line_start);
      geometry_msgs::msg::Point line_end;
      line_end.x = polygonStamped[i].polygon.points[j + 1].x;
      line_end.y = polygonStamped[i].polygon.points[j + 1].y;
      line_list.points.push_back(line_end);
    }
    // close loop for current polygon
    if (!polygonStamped[i].polygon.points.empty() &&
        polygonStamped[i].polygon.points.size() != 2) {
      geometry_msgs::msg::Point line_start;
      line_start.x = polygonStamped[i].polygon.points.back().x;
      line_start.y = polygonStamped[i].polygon.points.back().y;
      line_list.points.push_back(line_start);
      if (line_list.points.size() % 2 != 0) {
        geometry_msgs::msg::Point line_end;
        line_end.x = polygonStamped[i].polygon.points.front().x;
        line_end.y = polygonStamped[i].polygon.points.front().y;
        line_list.points.push_back(line_end);
      }
    }
  }
  marker_pub_->publish(line_list);
}

void CostmapStandaloneConversion::publishAsMarker(
    const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles) {
  visualization_msgs::msg::Marker line_list;
  line_list.header.frame_id = obstacles.header.frame_id;
  line_list.header.stamp = obstacles.header.stamp;
  line_list.ns = "Polygons";
  line_list.action = visualization_msgs::msg::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

  line_list.scale.x = 0.01;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;

  for (const auto &obstacle : obstacles.obstacles) {
    for (int j = 0; j < (int)obstacle.polygon.points.size() - 1; ++j) {
      geometry_msgs::msg::Point line_start;
      line_start.x = obstacle.polygon.points[j].x;
      line_start.y = obstacle.polygon.points[j].y;
      line_list.points.push_back(line_start);
      geometry_msgs::msg::Point line_end;
      line_end.x = obstacle.polygon.points[j + 1].x;
      line_end.y = obstacle.polygon.points[j + 1].y;
      line_list.points.push_back(line_end);
    }
    // close loop for current polygon
    if (!obstacle.polygon.points.empty() &&
        obstacle.polygon.points.size() != 2) {
      geometry_msgs::msg::Point line_start;
      line_start.x = obstacle.polygon.points.back().x;
      line_start.y = obstacle.polygon.points.back().y;
      line_list.points.push_back(line_start);
      if (line_list.points.size() % 2 != 0) {
        geometry_msgs::msg::Point line_end;
        line_end.x = obstacle.polygon.points.front().x;
        line_end.y = obstacle.polygon.points.front().y;
        line_list.points.push_back(line_end);
      }
    }
  }
  marker_pub_->publish(line_list);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(CostmapStandaloneConversion)