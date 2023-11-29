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

#ifndef POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_
#define POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_

#include <memory>
#include <optional>

#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_msgs/srv/polygon_service.hpp>
#include <polygon_coverage_msgs/msg/polygon_with_holes_stamped.hpp>
#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>
#include <polygon_coverage_planners/sensor_models/sensor_model_base.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <polygon_coverage_msgs/srv/planner_service.hpp>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace polygon_coverage_planning {

// A basic ros wrapper for planner in a 2D polynomial environment.
class PolygonPlannerBase {
 public:
  // Constructor
    // PolygonPlannerBase(const std::shared_ptr<rclcpp::Node> &nh,
    //                     const std::shared_ptr<rclcpp::Node> &nh_private);
  PolygonPlannerBase(std::shared_ptr<rclcpp::Node> &nh);

 protected:
  // Call to the actual planner.
  virtual bool solvePlanner(const Point_2& start, const Point_2& goal) = 0;
  // Reset the planner when a new polygon is set.
  virtual bool resetPlanner() = 0;
  // Publish the decomposition.
  virtual inline visualization_msgs::msg::MarkerArray createDecompositionMarkers()
      const {
    return visualization_msgs::msg::MarkerArray();
  }

  // Node handles
     std::shared_ptr<rclcpp::Node> &nh_;
    //  const std::shared_ptr<rclcpp::Node> &nh_private_;

  // The solution waypoints for a given start and goal.
  std::vector<Point_2> solution_;

  // Parameters
  std::optional<PolygonWithHoles> polygon_;
  // PolygonWithHoles polygon_;
  double wall_distance_;
  std::pair<PathCostFunction, CostFunctionType> path_cost_function_;
  std::optional<double> altitude_;
  bool latch_topics_;
  std::string global_frame_id_;
  bool publish_plan_on_planning_complete_;
  bool publish_visualization_on_planning_complete_;
  std::optional<double> v_max_;
  std::optional<double> a_max_;
  bool set_start_goal_from_rviz_;
  bool set_polygon_from_rviz_;
  std::optional<Point_2> start_;
  std::optional<Point_2> goal_;

 private:
  void test1(){
    RCLCPP_INFO(rclcpp::get_logger("coverage_planner111"), "2222333333.");
  }
  // Solve the planning problem. Stores status planning_complete_ and publishes
  // trajectory and visualization if enabled.
  void solve(const Point_2& start, const Point_2& goal);

  // Initial interactions with ROS
  void getParametersFromRos();
  void advertiseTopics();

  // Set a new polygon through a service call.
  bool setPolygonCallback(
   const std::shared_ptr<polygon_coverage_msgs::srv::PolygonService::Request> request,
      std::shared_ptr<polygon_coverage_msgs::srv::PolygonService::Response> response);

  // Set start and goal from clicked point.
  void clickPointCallback(const std::shared_ptr<geometry_msgs::msg::PointStamped> msg);
  // Set polygon from RVIZ polygon tool.
  void clickPolygonCallback(
      const std::shared_ptr<polygon_coverage_msgs::msg::PolygonWithHolesStamped> msg);
  // Solves the planning problem from start to goal.
  bool planPathCallback(
      const std::shared_ptr<polygon_coverage_msgs::srv::PlannerService::Request> request,
      std::shared_ptr<polygon_coverage_msgs::srv::PlannerService::Response> response);
  bool publishAllCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response);
  bool publishVisualizationCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  bool publishTrajectoryPointsCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                       std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // Visualization
  bool publishVisualization();

  // Publishing the plan
  bool publishTrajectoryPoints();

  void printMarkerArray(const visualization_msgs::msg::MarkerArray& array);
  // Publishers and Services
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_list_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<polygon_coverage_msgs::msg::PolygonWithHolesStamped>::SharedPtr polygon_sub_;
  rclcpp::Service<polygon_coverage_msgs::srv::PolygonService>::SharedPtr set_polygon_srv_;
  rclcpp::Service<polygon_coverage_msgs::srv::PlannerService>::SharedPtr plan_path_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_visualization_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_plan_points_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_all_srv_;

  // Planner status
  bool planning_complete_;

  visualization_msgs::msg::MarkerArray markers_;
};
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_
