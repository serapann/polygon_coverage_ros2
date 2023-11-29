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

#include "polygon_coverage_ros2/polygon_planner_base.h"
#include "polygon_coverage_ros2/ros_interface.h"

#include <functional>

#include <polygon_coverage_msgs/msg_from_xml_rpc.h>
#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>

#include <geometry_msgs/msg/pose_array.h>
#include <visualization_msgs/msg/marker_array.h>

#include "rclcpp/parameter_client.hpp"

namespace polygon_coverage_planning {

// PolygonPlannerBase::PolygonPlannerBase(const ros::NodeHandle& nh,
//                                        const ros::NodeHandle& nh_private)
PolygonPlannerBase::PolygonPlannerBase(std::shared_ptr<rclcpp::Node> &nh)
    : nh_(nh),
      // // nh_private_(nh_private),
      // wall_distance_(0.0),
      wall_distance_(1.0),
      path_cost_function_(
          {std::bind(&computeEuclideanPathCost, std::placeholders::_1),
           CostFunctionType::kDistance}),
      latch_topics_(true),
      global_frame_id_("world00000"),
      publish_plan_on_planning_complete_(false),
      publish_visualization_on_planning_complete_(true),
      set_start_goal_from_rviz_(false),
      set_polygon_from_rviz_(true),
      planning_complete_(false) 
      {
        nh_->declare_parameter("wall_distance", wall_distance_);
        nh_->declare_parameter("latch_topics", latch_topics_);
        nh_->declare_parameter("global_frame_id", global_frame_id_);
        nh_->declare_parameter("publish_plan_on_planning_complete", publish_plan_on_planning_complete_);
        nh_->declare_parameter("publish_visualization_on_planning_complete", publish_visualization_on_planning_complete_);
        nh_->declare_parameter("set_start_goal_from_rviz", set_start_goal_from_rviz_);
        nh_->declare_parameter("set_polygon_from_rviz", set_polygon_from_rviz_);
        // nh_->declare_parameter("planning_complete", planning_complete_);
        RCLCPP_INFO(rclcpp::get_logger("coverage_planner111"), "coverage_planner节点已经启动2222.");
  // Initial interactions with ROS
  getParametersFromRos();
  advertiseTopics();

  // Publish RVIZ.
  publishVisualization();
}
// void PolygonPlannerBase::test1(){
//   RCLCPP_INFO(rclcpp::get_logger("coverage_planner111"), "2222333333.");
// }
void PolygonPlannerBase::advertiseTopics() {
  // Advertising the visualization and planning messages
  marker_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "path_markers", 1);
  // waypoint_list_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseArray>(
  //     "waypoint_list", 1);
  // // Services for generating the plan.
  // set_polygon_srv_ = nh_->create_service<polygon_coverage_msgs::srv::PolygonService>(
  //     "set_polygon", std::bind(&PolygonPlannerBase::setPolygonCallback,this, std::placeholders::_1, std::placeholders::_2));
  // plan_path_srv_ = nh_->create_service<polygon_coverage_msgs::srv::PlannerService>(
  //     "plan_path", std::bind(&PolygonPlannerBase::planPathCallback,this, std::placeholders::_1, std::placeholders::_2));
  // //Services for performing publishing and visualization
  // publish_all_srv_ = nh_->create_service<std_srvs::srv::Empty>(
  //     "publish_all", std::bind(&PolygonPlannerBase::publishAllCallback,this, std::placeholders::_1, std::placeholders::_2));
  // publish_visualization_srv_ = nh_->create_service<std_srvs::srv::Empty>(
  //     "publish_visualization",
  //     std::bind(&PolygonPlannerBase::publishVisualizationCallback,this, std::placeholders::_1, std::placeholders::_2));
  // publish_plan_points_srv_ = nh_->create_service<std_srvs::srv::Empty>(
  //     "publish_path_points",
  //     std::bind(&PolygonPlannerBase::publishTrajectoryPointsCallback,this, std::placeholders::_1, std::placeholders::_2));

  // // Subscribe
  // clicked_point_sub_ = nh_->create_subscription<geometry_msgs::msg::PointStamped>(
  //     "/clicked_point", 1, std::bind(&PolygonPlannerBase::clickPointCallback, this, std::placeholders::_1));
  // polygon_sub_ = nh_->create_subscription<polygon_coverage_msgs::msg::PolygonWithHolesStamped>("/polygon", 1,
  //                              std::bind(&PolygonPlannerBase::clickPolygonCallback, this, std::placeholders::_1));
}

void PolygonPlannerBase::getParametersFromRos() {
  // Load the polygon from polygon message from parameter server.
  // The altitude and the global frame ID are set from the same message.
  // XmlRpc::XmlRpcValue polygon_xml_rpc;

  rclcpp::Logger logger = rclcpp::get_logger("PolygonPlannerBase::getParametersFromRos");
  // // 声明参数
  // nh_->declare_parameter<double>("polygon.holes.hole1.points.point1.x", 0.0);
  // nh_->declare_parameter<double>("polygon.holes.hole1.points.point1.y", 0.0);
  // nh_->declare_parameter<double>("polygon.hull.points.point1.x", 0.0);
  // nh_->declare_parameter<double>("polygon.hull.points.point1.y", 0.0);

  // // 使用 get_parameter 获取参数值
  // double hole1_point1_x, hole1_point1_y, hull_point1_x, hull_point1_y;

  // nh_->get_parameter("polygon.holes.hole1.points.point1.x", hole1_point1_x);
  // nh_->get_parameter("polygon.holes.hole1.points.point1.y", hole1_point1_y);
  // nh_->get_parameter("polygon.hull.points.point1.x", hull_point1_x);
  // nh_->get_parameter("polygon.hull.points.point1.y", hull_point1_y);

  // // 打印参数值
  // RCLCPP_INFO(nh_->get_logger(), "hole1.point1.x: %f", hole1_point1_x);
  // RCLCPP_INFO(nh_->get_logger(), "hole1.point1.y: %f", hole1_point1_y);
  // RCLCPP_INFO(nh_->get_logger(), "hull.point1.x: %f", hull_point1_x);
  // RCLCPP_INFO(nh_->get_logger(), "hull.point1.y: %f", hull_point1_y);

  polygon_coverage_msgs::msg::PolygonWithHolesStamped poly_msg;
  polygonWithHolesStampedMsgFromNode(nh_, &poly_msg);
  RCLCPP_INFO(nh_->get_logger(), "polygon.frame_id:%s",poly_msg.header.frame_id.c_str());
  RCLCPP_INFO(nh_->get_logger(), "polygon.hull.points.size:%d",poly_msg.polygon.hull.points.size());
  for (size_t i = 0; i < poly_msg.polygon.hull.points.size(); ++i)
  {
    RCLCPP_INFO(nh_->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i
    , poly_msg.polygon.hull.points[i].x
    , poly_msg.polygon.hull.points[i].y
    , poly_msg.polygon.hull.points[i].z);
  }
  RCLCPP_INFO(nh_->get_logger(), "polygon.holes.hole0.points.size:%d",poly_msg.polygon.holes[0].points.size());
  for (size_t i = 0; i < poly_msg.polygon.holes[0].points.size(); ++i)
  {
    RCLCPP_INFO(nh_->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i
    , poly_msg.polygon.holes[0].points[i].x
    , poly_msg.polygon.holes[0].points[i].y
    , poly_msg.polygon.holes[0].points[i].z);
  }
  RCLCPP_INFO(nh_->get_logger(), "polygon.holes.hole1.points.size:%d",poly_msg.polygon.holes[1].points.size());
  for (size_t i = 0; i < poly_msg.polygon.holes[1].points.size(); ++i)
  {
    RCLCPP_INFO(nh_->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i
    , poly_msg.polygon.holes[1].points[i].x
    , poly_msg.polygon.holes[1].points[i].y
    , poly_msg.polygon.holes[1].points[i].z);
  }


  PolygonWithHoles temp_pwh;
  double temp_alt;
  if (polygonFromMsg(poly_msg, &temp_pwh, &temp_alt, &global_frame_id_))
  {
    RCLCPP_INFO_STREAM(logger, "Successfully loaded polygon.");
    RCLCPP_INFO_STREAM(logger, "Altitude: " << temp_alt << " m");
    RCLCPP_INFO_STREAM(logger, "Global frame: " << global_frame_id_);
    RCLCPP_INFO_STREAM(logger, "Polygon:" << temp_pwh);
    polygon_ = std::make_optional(temp_pwh);
    altitude_ = std::make_optional(temp_alt);
  }

  // Getting control params from the server
  if (!nh_->get_parameter("wall_distance", wall_distance_)) {
    RCLCPP_WARN_STREAM(logger,"No wall distance specified. Using default value of: "
                    << wall_distance_);
  } else
    RCLCPP_INFO_STREAM(logger,"Wall distance: " << wall_distance_ << " m");

  // Cost function
  double temp_v_max = 0.0;
  nh_->declare_parameter("v_max", temp_v_max);
  if (nh_->get_parameter("v_max", temp_v_max)) {
    v_max_ = std::make_optional(temp_v_max);
  }
  double temp_a_max = 0.0;
  nh_->declare_parameter("a_max", temp_a_max);
  if (nh_->get_parameter("a_max", temp_a_max)) {
    a_max_ = std::make_optional(temp_a_max);
  }

  // Cost function type.
  int cost_function_type_int = static_cast<int>(path_cost_function_.second);
  nh_->declare_parameter("cost_function_type", cost_function_type_int);
  if (!nh_->get_parameter("cost_function_type", cost_function_type_int)) {
    RCLCPP_WARN_STREAM(logger,"No cost_function_type specified. Using default value of: "
                    << path_cost_function_.second << "("
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << ").");
  }
  if (!checkCostFunctionTypeValid(cost_function_type_int)) {
    RCLCPP_WARN_STREAM(logger,"cost_function_type not valid. Resetting to default: "
                    << path_cost_function_.second << "("
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << ").");
    cost_function_type_int = static_cast<int>(path_cost_function_.second);
  }
  path_cost_function_.second =
      static_cast<CostFunctionType>(cost_function_type_int);

  RCLCPP_INFO_STREAM(logger,
      "Cost function: " << getCostFunctionTypeName(path_cost_function_.second));
  if (path_cost_function_.second == CostFunctionType::kTime) {
    if (!v_max_.has_value() || !a_max_.has_value()) {
        if(!v_max_.has_value()){
            RCLCPP_WARN(logger, "Velocity 'v_max' not set.");
        }
        if(!a_max_.has_value()){
            RCLCPP_WARN(logger, "Acceleration 'a_max' not set.");
        }
      RCLCPP_WARN(logger,"Falling back to distance cost function.");
      path_cost_function_.second = CostFunctionType::kDistance;
    } else {
      RCLCPP_INFO_STREAM(logger,"v_max: " << v_max_.value()
                                << ", a_max: " << a_max_.value());
    }
  }

  switch (path_cost_function_.second) {
    case CostFunctionType::kDistance: {
      path_cost_function_.first =
          std::bind(&computeEuclideanPathCost, std::placeholders::_1);
      break;
    }
    case CostFunctionType::kTime: {
      path_cost_function_.first =
          std::bind(&computeVelocityRampPathCost, std::placeholders::_1,
                    v_max_.value(), a_max_.value());
      break;
    }
    case CostFunctionType::kWaypoints: {
      path_cost_function_.first =
          std::bind(&computeWaypointsPathCost, std::placeholders::_1);
      break;
    }
    default: {
      RCLCPP_WARN_STREAM(logger,"Cost function type: "
                      << getCostFunctionTypeName(path_cost_function_.second)
                      << "not implemented. Using euclidean distance.");
      break;
    }
  }

  // Getting the behaviour flags
  nh_->get_parameter("latch_topics", latch_topics_);
  nh_->get_parameter("publish_plan_on_planning_complete",
                       publish_plan_on_planning_complete_);
  nh_->get_parameter("publish_visualization_on_planning_complete",
                       publish_visualization_on_planning_complete_);
  // nh_->get_parameter("global_frame_id", global_frame_id_);//一旦失败，又使用默认值，这行无意义了
  nh_->get_parameter("set_start_goal_from_rviz", set_start_goal_from_rviz_);
  nh_->get_parameter("set_polygon_from_rviz", set_polygon_from_rviz_);

  RCLCPP_INFO_STREAM(logger,"latch_topics: " << latch_topics_);
  RCLCPP_INFO_STREAM(logger,"publish_plan_on_planning_complete: " << publish_plan_on_planning_complete_);
  RCLCPP_INFO_STREAM(logger,"publish_visualization_on_planning_complete: " << publish_visualization_on_planning_complete_);
  RCLCPP_INFO_STREAM(logger,"global_frame_id: " << global_frame_id_);
  RCLCPP_INFO_STREAM(logger,"set_start_goal_from_rviz: " << set_start_goal_from_rviz_);
  RCLCPP_INFO_STREAM(logger,"set_polygon_from_rviz: " << set_polygon_from_rviz_);
}

void PolygonPlannerBase::solve(const Point_2& start, const Point_2& goal) {
  // rclcpp::Logger logger = rclcpp::get_logger("PolygonPlannerBase::solve");
  // RCLCPP_INFO_STREAM(logger,"Start solving.");
  // if ((planning_complete_ = solvePlanner(start, goal))) {
  //   RCLCPP_INFO_STREAM(logger,"Finished plan."
  //                   << std::endl
  //                   << "Optimization Criterion: "
  //                   << getCostFunctionTypeName(path_cost_function_.second)
  //                   << std::endl
  //                   << "Number of waypoints: " << solution_.size() << std::endl
  //                   << "Start point: " << start << std::endl
  //                   << "Goal point: " << goal << std::endl
  //                   << "Altitude: " << altitude_.value() << " [m]" << std::endl
  //                   << "Path length: " << computeEuclideanPathCost(solution_)
  //                   << " [m]");
  //   if (v_max_.has_value() && a_max_.has_value())
  //     RCLCPP_INFO_STREAM(logger,"Path time: "
  //                     << computeVelocityRampPathCost(solution_, v_max_.value(),
  //                                                    a_max_.value())
  //                     << " [s]");

  //   // Publishing the plan if requested
  //   if (publish_plan_on_planning_complete_) {
  //     publishTrajectoryPoints();
  //   }
  //   // Publishing the visualization if requested
  //   if (publish_visualization_on_planning_complete_) {
  //     publishVisualization();
  //   }
  // } else {
  //   RCLCPP_ERROR_STREAM(logger,"Failed calculating plan.");
  // }
}

bool PolygonPlannerBase::publishVisualization() {
  rclcpp::Logger logger = rclcpp::get_logger("PolygonPlannerBase::publishVisualization");
  RCLCPP_INFO_STREAM(logger,"Sending visualization messages.");

  // Delete old markers.
  // for(visualization_msgs::msg::Marker& m : markers_.markers)
  //   m.action = visualization_msgs::msg::Marker::DELETE;
  for (size_t i = 0; i < markers_.markers.size(); ++i) {
    markers_.markers[i].action = visualization_msgs::msg::Marker::DELETE;
  }
  marker_pub_->publish(markers_);

  // Create new markers.
  markers_.markers.clear();
  // The planned path:
  visualization_msgs::msg::Marker path_points, path_line_strips;
  const double kPathLineSize = 0.2;
  const double kPathPointSize = 0.2;
  // if (!altitude_.has_value()) {
  //   RCLCPP_WARN_STREAM(logger,"Cannot send visualization because altitude not set.");
  //   return false;
  // }

  // if (!planning_complete_) {
  //   RCLCPP_WARN_STREAM(logger,
  //       "Cannot send solution visualization because plan has not been made.");
  // } else {
  //   createMarkers(solution_, altitude_.value(), global_frame_id_,
  //                 "vertices_and_strip", Color::Gray(), Color::Gray(),
  //                 kPathLineSize, kPathPointSize, &path_points,
  //                 &path_line_strips);
  //   markers_.markers.push_back(path_points);
  //   markers_.markers.push_back(path_line_strips);

  //   // Start and end points
  //   visualization_msgs::msg::Marker start_point, end_point;
  //   createStartAndEndPointMarkers(solution_.front(), solution_.back(),
  //                                 altitude_.value(), global_frame_id_, "points",
  //                                 &start_point, &end_point);
  //   markers_.markers.push_back(start_point);
  //   markers_.markers.push_back(end_point);

  //   // Start and end text.
  //   visualization_msgs::msg::Marker start_text, end_text;
  //   createStartAndEndTextMarkers(solution_.front(), solution_.back(),
  //                                altitude_.value(), global_frame_id_, "points",
  //                                &start_text, &end_text);
  //   markers_.markers.push_back(start_text);
  //   markers_.markers.push_back(end_text);
  // }

  // The original polygon:
  const double kPolygonLineSize = 0.4;
  visualization_msgs::msg::MarkerArray polygon;
  // if (!polygon_.has_value()) {
  //   RCLCPP_WARN_STREAM(logger,"Cannot send visualization because polygon not set.");
  //   return false;
  // }

// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Polygon_with_holes_2.h>
// #include <CGAL/Polygon_2.h>
// #include <vector>

// // 使用 CGAL 的内核和点类型
// typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// typedef K::Point_2 Point_2;
// typedef CGAL::Polygon_2<K> Polygon_2;
// typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles;

// // 创建一个外部多边形
// Polygon_2 outer;
// outer.push_back(Point_2(0.0, 0.0));
// outer.push_back(Point_2(100.0, 0.0));
// outer.push_back(Point_2(100.0, 100.0));
// outer.push_back(Point_2(0.0, 100.0));

// // 创建孔洞
// Polygon_2 hole1;
// hole1.push_back(Point_2(55.66013134894843, 55.65120076448767));
// hole1.push_back(Point_2(45.0961855134036, 25.92900939815026));
// hole1.push_back(Point_2(73.0, 34.24070455081452));
// hole1.push_back(Point_2(73.0, 38.76409597325997));

// Polygon_2 hole2;
// hole2.push_back(Point_2(10.532926709487015, 65.1078154035655));
// hole2.push_back(Point_2(35.567417803658135, 50.95927935884282));
// hole2.push_back(Point_2(22.00327751845971, 84.36805971116891));

// // 创建 PolygonWithHoles 对象并填充外部多边形和孔洞
// polygon_.outer_boundary() = outer;

// // 添加孔洞到多边形
// polygon_.add_hole(hole1);
// polygon_.add_hole(hole2);

//   double altitude = 0.5;
//   std::string frame_id = "world";
//   std::string ns = "polygon";
//   Color polygon_color;
//   polygon_color.r = 0.0; // 绿色
//   polygon_color.g = 1.0;
//   polygon_color.b = 0.0;
//   polygon_color.a = 1.0; // 不透明

//   Color hole_color;
//   hole_color.r = 1.0; // 红色
//   hole_color.g = 0.0;
//   hole_color.b = 0.0;
//   hole_color.a = 1.0; // 不透明

//   double line_size = 0.05; // 线段大小
//   double point_size = 0.1; // 点的大小
//   visualization_msgs::msg::MarkerArray array;
//   createPolygonMarkers(polygon_, altitude, frame_id, ns, polygon_color, hole_color, line_size, point_size, &array);
//   RCLCPP_WARN_STREAM(logger,"MarkerArray: ");
//   RCLCPP_INFO(logger, "MarkerArray: %s", array);
//   printMarkerArray(array);

//   RCLCPP_WARN_STREAM(logger,"polygon set success.");

  createPolygonMarkers(polygon_.value(), altitude_.value(), global_frame_id_,
                       "polygon", Color::Black(), Color::Black(),
                       kPolygonLineSize, kPolygonLineSize, &polygon);
  markers_.markers.insert(markers_.markers.end(), polygon.markers.begin(),
                          polygon.markers.end());
  // markers_.markers.insert(markers_.markers.end(), array.markers.begin(),
  //                         array.markers.end());
  RCLCPP_WARN_STREAM(logger,"polygon set success111.");

  // The decomposed polygons.
  visualization_msgs::msg::MarkerArray decomposition_markers =
      createDecompositionMarkers();
  markers_.markers.insert(markers_.markers.end(),
                          decomposition_markers.markers.begin(),
                          decomposition_markers.markers.end());

  // visualization_msgs::msg::MarkerArray marker_array;
  // // 创建第一个标记
  // visualization_msgs::msg::Marker marker1;
  // marker1.header.frame_id = "world";
  // marker1.ns = "markers";
  // marker1.id = 1;
  // marker1.type = visualization_msgs::msg::Marker::SPHERE;
  // marker1.action = visualization_msgs::msg::Marker::ADD;
  // marker1.pose.position.x = 1.0;
  // marker1.pose.position.y = 2.0;
  // marker1.pose.position.z = 3.0;
  // marker1.pose.orientation.x = 0.0;
  // marker1.pose.orientation.y = 0.0;
  // marker1.pose.orientation.z = 0.0;
  // marker1.pose.orientation.w = 1.0;
  // marker1.scale.y = 0.5;
  // marker1.scale.z = 0.5;
  // marker1.color.r = 1.0;
  // marker1.color.g = 0.0;
  // marker1.color.b = 0.0;
  // marker1.color.a = 1.0;
  // marker1.lifetime = rclcpp::Duration(10, 0);  // 持续时间为10秒
  // marker_array.markers.push_back(marker1);
  // Publishing 
  // marker_pub_->publish(marker_array);
  // marker_pub_->publish(markers_);
  while(rclcpp::ok()){
    // marker_pub_->publish(marker_array);
    // if (!markers_->markers.empty()) {
    //    marker_pub_->publish(markers_);
    //    break;
    // }
    marker_pub_->publish(markers_);
  }

  // Success
  return true;
}

void PolygonPlannerBase::printMarkerArray(const visualization_msgs::msg::MarkerArray& array) {
for (const auto& marker : array.markers) {
    RCLCPP_INFO(rclcpp::get_logger("marker_array_printer"), "Marker ID: %d", marker.id);
    RCLCPP_INFO(rclcpp::get_logger("marker_array_printer"), "Points:");
    for (const auto& point : marker.points) {
      RCLCPP_INFO(rclcpp::get_logger("marker_array_printer"), "x: %f, y: %f, z: %f", point.x, point.y, point.z);
    }
    // 继续打印其他字段...
  }
}

bool PolygonPlannerBase::publishTrajectoryPoints() {
  // rclcpp::Logger logger = rclcpp::get_logger("PolygonPlannerBase::ppublishTrajectoryPoints");
  // if (!planning_complete_) {
  //   RCLCPP_WARN(logger,"Cannot send trajectory messages because plan has not been made.");
  //   return false;
  // }
  // RCLCPP_INFO_STREAM(logger,"Sending trajectory messages");

  // // Convert path to pose array.
  // geometry_msgs::msg::PoseArray trajectory_points_pose_array;
  // if (!altitude_.has_value()) {
  //   RCLCPP_WARN_STREAM(logger,"Cannot send trajectory because altitude not set.");
  //   return false;
  // }
  // poseArrayMsgFromPath(solution_, altitude_.value(), global_frame_id_,
  //                      &trajectory_points_pose_array);

  // // Publishing
  // waypoint_list_pub_->publish(trajectory_points_pose_array);

  // // Success
  return true;
}

// bool PolygonPlannerBase::setPolygonCallback(
//     polygon_coverage_msgs::srv::PolygonService::Request& request,
//     polygon_coverage_msgs::srv::PolygonService::Response& response) {
bool PolygonPlannerBase::setPolygonCallback(
    const std::shared_ptr<polygon_coverage_msgs::srv::PolygonService::Request> request,
    std::shared_ptr<polygon_coverage_msgs::srv::PolygonService::Response> response) {        
  // rclcpp::Logger logger = rclcpp::get_logger("PolygonPlannerBase::setPolygonCallback");
  // PolygonWithHoles temp_pwh;
  // double temp_alt;
  // if (!polygonFromMsg(request->polygon, &temp_pwh, &temp_alt,
  //                     &global_frame_id_)) {
  //   RCLCPP_ERROR_STREAM(logger,"Failed loading correct polygon.");
  //   RCLCPP_ERROR_STREAM(logger,"Planner is in an invalid state.");
  //   polygon_.reset();
  //   return false;
  // }
  // polygon_ = std::make_optional(temp_pwh);
  // altitude_ = std::make_optional(temp_alt);

  // RCLCPP_INFO_STREAM(logger,"Successfully loaded polygon.");
  // RCLCPP_INFO_STREAM(logger,"Altitude: " << altitude_.value() << "m");
  // RCLCPP_INFO_STREAM(logger,"Global frame: " << global_frame_id_);
  // RCLCPP_INFO_STREAM(logger,"Polygon:" << polygon_.value());

  // response->success = resetPlanner();
  return true;  // Still return true to identify service has been reached.
}

bool PolygonPlannerBase::planPathCallback(
    const std::shared_ptr<polygon_coverage_msgs::srv::PlannerService::Request> request,
    std::shared_ptr<polygon_coverage_msgs::srv::PlannerService::Response> response) {
  // rclcpp::Logger logger = rclcpp::get_logger("PolygonPlannerBase::planPathCallback");
  // planning_complete_ = false;
  // if (!polygon_.has_value()) {
  //   RCLCPP_WARN(logger,"Polygon not set. Cannot plan path.");
  //   response->success = planning_complete_;
  //   return true;
  // }
  // const Point_2 start(request->start_pose.pose.position.x,
  //                     request->start_pose.pose.position.y);
  // const Point_2 goal(request->goal_pose.pose.position.x,
  //                    request->goal_pose.pose.position.y);
  // solve(start, goal);  // Calculate optimal path.
  // if (altitude_.has_value()) {
  //   msgMultiDofJointTrajectoryFromPath(solution_, altitude_.value(),
  //                                      &response->sampled_plan);
  // } else {
  //   RCLCPP_WARN(logger,"Cannot plan path. Altitude not set.");
  // }
  // response->success = planning_complete_;
  return true;
}

bool PolygonPlannerBase::publishAllCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  // bool success_publish_trajectory = publishTrajectoryPoints();
  // bool success_publish_visualization = publishVisualization();
  // return (success_publish_trajectory && success_publish_visualization);
  return true;
}

bool PolygonPlannerBase::publishVisualizationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  return publishVisualization();
}

bool PolygonPlannerBase::publishTrajectoryPointsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  return publishTrajectoryPoints();
}

// Reset the planner when a new polygon is set.
bool PolygonPlannerBase::resetPlanner() {
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("PolygonPlannerBase::resetPlanner"),
                      "resetPlanner is not implemented.");
  return false;
}

void PolygonPlannerBase::clickPointCallback(
    const std::shared_ptr<geometry_msgs::msg::PointStamped> msg) {
  // if (!set_start_goal_from_rviz_) return;

  // if (!start_.has_value()) {
  //   RCLCPP_INFO(rclcpp::get_logger("PolygonPlannerBase::clickPointCallback"),"Selecting START from RVIZ PublishPoint tool.");
  //   start_ = std::make_optional<Point_2>(msg->point.x, msg->point.y);
  // } else if (!goal_.has_value()) {
  //   RCLCPP_INFO(rclcpp::get_logger("PolygonPlannerBase::clickPointCallback"),"Selecting GOAL from RVIZ PublishPoint tool.");
  //   goal_ = std::make_optional<Point_2>(msg->point.x, msg->point.y);
  // }

  // if (start_.has_value() && goal_.has_value()) {
  //   solve(start_.value(), goal_.value());
  //   start_.reset();
  //   goal_.reset();
  // }

  return;
}

void PolygonPlannerBase::clickPolygonCallback(
    const std::shared_ptr<polygon_coverage_msgs::msg::PolygonWithHolesStamped> msg) {
  // rclcpp::Logger logger = rclcpp::get_logger("PolygonPlannerBase::clickPolygonCallback");
  // if (!set_polygon_from_rviz_) return;

  // RCLCPP_INFO(logger,"Updating polygon from RVIZ polygon tool.");
  // PolygonWithHoles temp_pwh;
  // double temp_alt;
  // // if (polygonFromMsg(msg, &temp_pwh, &temp_alt, &global_frame_id_)) {
  //   if (polygonFromMsg(*msg.get(), &temp_pwh, &temp_alt, &global_frame_id_)) {
  //   RCLCPP_INFO_STREAM(logger,"Successfully loaded polygon.");
  //   RCLCPP_INFO_STREAM(logger,"Altitude: " << temp_alt << " m");
  //   RCLCPP_INFO_STREAM(logger,"Global frame: " << global_frame_id_);
  //   RCLCPP_INFO_STREAM(logger,"Polygon:" << temp_pwh);
  //   polygon_ = std::make_optional(temp_pwh);
  //   altitude_ = std::make_optional(temp_alt);
  // }

  // planning_complete_ = false;
  // resetPlanner();
  // publishVisualization();

  return;
}

}  // namespace polygon_coverage_planning
