#include "rclcpp/rclcpp.hpp"
#include"test1/test1.h"
#include <optional>
#include "rclcpp/parameter_client.hpp"
#include "geometry_msgs/msg/point.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test1"); 
  RCLCPP_INFO(rclcpp::get_logger("test1"), "test1节点已经启动.");

  // auto planner = std::make_shared<polygon_coverage_planning::CoveragePlanner<polygon_coverage_planning::PolygonStripmapPlanner>>(node);
//   auto test = std::make_shared<test::test1>(node);
  test::test1 test(node);

  // rclcpp::SyncParametersClient parameter_client(node);

        // Define the namespace for the parameters
  // const std::string param_namespace = "coverage_planner";
  
// Get parameters from the YAML file
  // int decomposition_type;
  // int cost_function_type;
  // double v_max;
  // double a_max;
  // double wall_distance;
  // bool offset_polygons;
  // bool sweep_single_direction;
  // int sensor_model_type;
  // double lateral_overlap;
  // double lateral_footprint;
  // double lateral_fov;
  // bool latch_topics;
  // bool publish_plan_on_planning_complete;
  // bool publish_visualization_on_planning_complete;
  // bool set_start_goal_from_rviz;
  // bool set_polygon_from_rviz;

  enum DecompositionType {
    kBCD = 0,  // Boustrophedon.
    kTCD       // Trapezoidal.
  };
  enum SensorModelType { kLine = 0, kFrustum };
  DecompositionType decomposition_type_;
  int cost_function_type_1;
  int cost_function_type_2;
  int cost_function_type_3;
  std::optional<double> v_max_;
  std::optional<double> a_max_;
  double wall_distance_;
  bool offset_polygons_;
  bool sweep_single_direction_;
  SensorModelType sensor_model_type_;
  std::optional<double> lateral_overlap_;
  std::optional<double> lateral_footprint_;
  std::optional<double> lateral_fov_;
  bool latch_topics_;
  bool publish_plan_on_planning_complete_;
  bool publish_visualization_on_planning_complete_;
  bool set_start_goal_from_rviz_;
  bool set_polygon_from_rviz_;
  node->declare_parameter("cost_function_type", 2);
  // node->get_parameter("coverage_planner.decomposition_type", decomposition_type_);
  node->get_parameter("cost_function_type", cost_function_type_1);
  // node->get_parameter("/test1_node.cost_function_type", cost_function_type_2);
  cost_function_type_3 = node->get_parameter("cost_function_type").get_value<int>();
  // parameter_client.get_parameter("cost_function_type", cost_function_type_); 
  // node->get_parameter("coverage_planner.v_max", v_max);
  // node->get_parameter("coverage_planner.a_max", a_max);
  // node->get_parameter("coverage_planner.wall_distance", wall_distance);
  // node->get_parameter("coverage_planner.offset_polygons", offset_polygons);
  // node->get_parameter("coverage_planner.sweep_single_direction", sweep_single_direction);
  // node->get_parameter("coverage_planner.sensor_model_type", sensor_model_type);
  // node->get_parameter("coverage_planner.lateral_overlap", lateral_overlap);
  // node->get_parameter("coverage_planner.lateral_footprint", lateral_footprint);
  // node->get_parameter("coverage_planner.lateral_fov", lateral_fov);
  // node->get_parameter("coverage_planner.latch_topics", latch_topics);
  // node->get_parameter("coverage_planner.publish_plan_on_planning_complete", publish_plan_on_planning_complete);
  // node->get_parameter("coverage_planner.publish_visualization_on_planning_complete", publish_visualization_on_planning_complete);
  // node->get_parameter("coverage_planner.set_start_goal_from_rviz", set_start_goal_from_rviz);
  // node->get_parameter("coverage_planner.set_polygon_from_rviz", set_polygon_from_rviz);

  // Use the obtained parameters
  // You can print or use the parameters as needed
  // RCLCPP_INFO(node->get_logger(), "decomposition_type: %d", decomposition_type_);
  RCLCPP_INFO(node->get_logger(), "cost_function_type_1: %d", cost_function_type_1);
  // RCLCPP_INFO(node->get_logger(), "cost_function_type_2: %d", cost_function_type_2);
  RCLCPP_INFO(node->get_logger(), "cost_function_type_3: %d", cost_function_type_3);
  // RCLCPP_INFO(node->get_logger(), "v_max: %f", v_max);
  // RCLCPP_INFO(node->get_logger(), "a_max: %f", a_max);
  // RCLCPP_INFO(node->get_logger(), "wall_distance: %f", wall_distance);
  // RCLCPP_INFO(node->get_logger(), "offset_polygons: %d", offset_polygons);
  // RCLCPP_INFO(node->get_logger(), "sweep_single_direction: %d", sweep_single_direction);
  // RCLCPP_INFO(node->get_logger(), "sensor_model_type: %d", sensor_model_type);
  // RCLCPP_INFO(node->get_logger(), "lateral_overlap: %f", lateral_overlap);
  // RCLCPP_INFO(node->get_logger(), "lateral_footprint: %f", lateral_footprint);
  // RCLCPP_INFO(node->get_logger(), "lateral_fov: %f", lateral_fov);
  // RCLCPP_INFO(node->get_logger(), "latch_topics: %d", latch_topics);
  // RCLCPP_INFO(node->get_logger(), "publish_plan_on_planning_complete: %d", publish_plan_on_planning_complete);
  // RCLCPP_INFO(node->get_logger(), "publish_visualization_on_planning_complete: %d", publish_visualization_on_planning_complete);
  // RCLCPP_INFO(node->get_logger(), "set_start_goal_from_rviz: %d", set_start_goal_from_rviz);
  // RCLCPP_INFO(node->get_logger(), "set_polygon_from_rviz: %d", set_polygon_from_rviz);  

  // 声明参数
  node->declare_parameter<std::string>("polygon.header.frame_id", "world111");
  node->declare_parameter<double>("polygon.holes.hole1.points.point1.x", 0.0);
  node->declare_parameter<double>("polygon.holes.hole1.points.point1.y", 0.0);
  node->declare_parameter<double>("polygon.hull.points.point1.x", 0.0);
  node->declare_parameter<double>("polygon.hull.points.point1.y", 0.0);

  // 使用 get_parameter 获取参数值
  double hole1_point1_x, hole1_point1_y, hull_point1_x, hull_point1_y;
  std::string frame_id;
  node->get_parameter("polygon.header.frame_id", frame_id);
  node->get_parameter("polygon.holes.hole1.points.point1.x", hole1_point1_x);
  node->get_parameter("polygon.holes.hole1.points.point1.y", hole1_point1_y);
  node->get_parameter("polygon.hull.points.point1.x", hull_point1_x);
  node->get_parameter("polygon.hull.points.point1.y", hull_point1_y);

  // 打印参数值
  RCLCPP_INFO(node->get_logger(), "Frame ID: %s", frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "hole1.point1.x: %f", hole1_point1_x);
  RCLCPP_INFO(node->get_logger(), "hole1.point1.y: %f", hole1_point1_y);
  RCLCPP_INFO(node->get_logger(), "hull.point1.x: %f", hull_point1_x);
  RCLCPP_INFO(node->get_logger(), "hull.point1.y: %f", hull_point1_y);

  // Declare the parameter
  std::vector<std::string> parameterValue = {
    "x: 0, y: 0, z: 0",
    "x: 0, y: 0, z: 0",
    "x: 0, y: 0, z: 0",
    "x: 0, y: 0, z: 0"
  };
  node->declare_parameter("polygon.hull.points", parameterValue);
  // Get the parameter as a vector of strings
  std::vector<std::string> retrievedValue;
  if (node->get_parameter("polygon.hull.points", retrievedValue)) {
    for (const std::string& pointStr : retrievedValue) {
      geometry_msgs::msg::Point point;

      // Split the string by commas to separate key-value pairs
      std::vector<std::string> keyValuePairs;
      std::istringstream tokenStream(pointStr);
      std::string token;
      while (std::getline(tokenStream, token, ',')) {
        keyValuePairs.push_back(token);
      }

      // Parse key-value pairs
      for (const std::string& keyValue : keyValuePairs) {
        size_t pos = keyValue.find(":");
        if (pos != std::string::npos) {
          std::string key = keyValue.substr(0, pos);
          std::string value = keyValue.substr(pos + 1);

          // Remove leading/trailing whitespace
          key = key.substr(key.find_first_not_of(" "), key.find_last_not_of(" ") + 1);
          value = value.substr(value.find_first_not_of(" "), value.find_last_not_of(" ") + 1);

          if (key == "x") {
            point.x = std::stod(value);
          } else if (key == "y") {
            point.y = std::stod(value);
          } else if (key == "z") {
            point.z = std::stod(value);
          }
        }
      }

      // Now you can work with the geometry_msgs::msg::Point
      RCLCPP_INFO(node->get_logger(), "x=%f, y=%f, z=%f", point.x, point.y, point.z);
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to get the parameter.");
  }
  // Spin (and process service calls)
  rclcpp::spin(node);
  // Shutdown ROS 2 gracefully
  rclcpp::shutdown();
  // Exit tranquilly
  return 0;
}