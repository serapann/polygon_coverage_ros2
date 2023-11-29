/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rclcpp/rclcpp.hpp"

#include <polygon_coverage_planners/planners/polygon_stripmap_planner.h>
#include "polygon_coverage_ros2/coverage_planner.h"

// Standard C++ entry point
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
// Create a ROS 2 node
  auto node = std::make_shared<rclcpp::Node>("coverage_planner"); 
  RCLCPP_INFO(rclcpp::get_logger("coverage_planner111"), "coverage_planner节点已经启动.");
  // try
  // {
  //   YAML::Node config = YAML::LoadFile("install/polygon_coverage_ros2/share/polygon_coverage_ros2/cfg/coverage_planner.yaml");
  //   YAML::Node coverage_planner_params = config["coverage_planner"]["ros__parameters"];

  //   int decomposition_type = coverage_planner_params["decomposition_type"].as<int>();
  //   int cost_function_type = coverage_planner_params["cost_function_type"].as<int>();
  //   double v_max = coverage_planner_params["v_max"].as<double>();
  //   double a_max = coverage_planner_params["a_max"].as<double>();
  //   double wall_distance = coverage_planner_params["wall_distance"].as<double>();
  //   bool offset_polygons = coverage_planner_params["offset_polygons"].as<bool>();
  //   bool sweep_single_direction = coverage_planner_params["sweep_single_direction"].as<bool>();

  //   int sensor_model_type = coverage_planner_params["sensor_model_type"].as<int>();
  //   double lateral_overlap = coverage_planner_params["lateral_overlap"].as<double>();
  //   double lateral_footprint = coverage_planner_params["lateral_footprint"].as<double>();
  //   double lateral_fov = coverage_planner_params["lateral_fov"].as<double>();

  //   bool latch_topics = coverage_planner_params["latch_topics"].as<bool>();
  //   bool publish_plan_on_planning_complete = coverage_planner_params["publish_plan_on_planning_complete"].as<bool>();
  //   bool publish_visualization_on_planning_complete = coverage_planner_params["publish_visualization_on_planning_complete"].as<bool>();
  //   bool set_start_goal_from_rviz = coverage_planner_params["set_start_goal_from_rviz"].as<bool>();
  //   bool set_polygon_from_rviz = coverage_planner_params["set_polygon_from_rviz"].as<bool>();

  //   // 使用获取的参数
  //   RCLCPP_INFO(node->get_logger(), "decomposition_type: %d", decomposition_type);
  //   RCLCPP_INFO(node->get_logger(), "cost_function_type: %d", cost_function_type);
  //   RCLCPP_INFO(node->get_logger(), "v_max: %f", v_max);
  //   RCLCPP_INFO(node->get_logger(), "a_max: %f", a_max);
  //   RCLCPP_INFO(node->get_logger(), "wall_distance: %f", wall_distance);
  //   RCLCPP_INFO(node->get_logger(), "offset_polygons: %s", offset_polygons ? "true" : "false");
  //   RCLCPP_INFO(node->get_logger(), "sweep_single_direction: %s", sweep_single_direction ? "true" : "false");

  //   RCLCPP_INFO(node->get_logger(), "sensor_model_type: %d", sensor_model_type);
  //   RCLCPP_INFO(node->get_logger(), "lateral_overlap: %f", lateral_overlap);
  //   RCLCPP_INFO(node->get_logger(), "lateral_footprint: %f", lateral_footprint);
  //   RCLCPP_INFO(node->get_logger(), "lateral_fov: %f", lateral_fov);

  //   RCLCPP_INFO(node->get_logger(), "latch_topics: %s", latch_topics ? "true" : "false");
  //   RCLCPP_INFO(node->get_logger(), "publish_plan_on_planning_complete: %s", publish_plan_on_planning_complete ? "true" : "false");
  //   RCLCPP_INFO(node->get_logger(), "publish_visualization_on_planning_complete: %s", publish_visualization_on_planning_complete ? "true" : "false");
  //   RCLCPP_INFO(node->get_logger(), "set_start_goal_from_rviz: %s", set_start_goal_from_rviz ? "true" : "false");
  //   RCLCPP_INFO(node->get_logger(), "set_polygon_from_rviz: %s", set_polygon_from_rviz ? "true" : "false");
  // }
  // catch (const YAML::Exception &e)
  // {
  //   RCLCPP_ERROR(node->get_logger(), "YAML Error: %s", e.what());
  // }

  // try
  // {
  //   // 读取YAML文件
  //   YAML::Node config = YAML::LoadFile("install/polygon_coverage_ros2/share/polygon_coverage_ros2/cfg/polygons/example_polygon_epfl_simple.yaml");

  //   // 获取coverage_planner参数节点
  //   YAML::Node coverage_planner_params = config["coverage_planner"]["ros__parameters"];

  //   // 获取polygon参数节点
  //   YAML::Node polygon_params = coverage_planner_params["polygon"];

  //   // 获取polygon参数的header
  //   YAML::Node header_params = polygon_params["header"];
  //   int seq = header_params["seq"].as<int>();
  //   int secs = header_params["stamp"]["secs"].as<int>();
  //   int nsecs = header_params["stamp"]["nsecs"].as<int>();
  //   std::string frame_id = header_params["frame_id"].as<std::string>();

  //   RCLCPP_INFO(node->get_logger(), "seq: %d", seq);
  //   RCLCPP_INFO(node->get_logger(), "secs: %d", secs);
  //   RCLCPP_INFO(node->get_logger(), "nsecs: %d", nsecs);
  //   RCLCPP_INFO(node->get_logger(), "frame_id: %s", frame_id.c_str());

  //   // 获取holes参数节点
  //   YAML::Node holes_params = polygon_params["holes"];
  //   for (const YAML::Node &hole : holes_params)
  //   {
  //     // 获取points参数节点
  //     YAML::Node points_params = hole["points"];
  //     for (const YAML::Node &point : points_params)
  //     {
  //       double x = point["x"].as<double>();
  //       double y = point["y"].as<double>();
  //       double z = point["z"].as<double>();
  //       RCLCPP_INFO(node->get_logger(), "Hole Point (x, y, z): (%f, %f, %f)", x, y, z);
  //     }
  //   }

  //   // 获取hull参数节点
  //   YAML::Node hull_params = polygon_params["hull"]["points"];
  //   for (const YAML::Node &point : hull_params)
  //   {
  //     double x = point["x"].as<double>();
  //     double y = point["y"].as<double>();
  //     double z = point["z"].as<double>();
  //     RCLCPP_INFO(node->get_logger(), "Hull Point (x, y, z): (%f, %f, %f)", x, y, z);
  //   }
  // }
  // catch (const YAML::Exception &e)
  // {
  //   RCLCPP_ERROR(node->get_logger(), "YAML Error: %s", e.what());
  // }
  // Creating the coverage planner with ros interface
  // polygon_coverage_planning::CoveragePlanner<
  //     polygon_coverage_planning::PolygonStripmapPlanner>
  //     planner(nh, nh_private);
  // Create the coverage planner with ROS 2 interface
  auto planner = std::make_shared<polygon_coverage_planning::CoveragePlanner<polygon_coverage_planning::PolygonStripmapPlanner>>(node);
    // auto planner = std::make_shared<polygon_coverage_planning::CoveragePlanner>(node);
  // Spin (and process service calls)
  rclcpp::spin(node);
  // Shutdown ROS 2 gracefully
  rclcpp::shutdown();
  // Exit tranquilly
  return 0;
}
