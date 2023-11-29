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

#include "polygon_coverage_msgs/msg_from_xml_rpc.h"

#include <exception>
#include <string>

// #include <geometry_msgs/msg/point32.hpp>
// #include <ros/assert.h>
// #include <ros/console.h>
// #include <ros/time.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Header.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Time.h>
// #include <std_msgs/UInt32.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

// #include <polygon_coverage_msgs/PolygonWithHoles.h>
// #include <polygon_coverage_msgs/PolygonWithHolesStamped.h>
#include <polygon_coverage_msgs/msg/polygon_with_holes.hpp>
#include <polygon_coverage_msgs/msg/polygon_with_holes_stamped.hpp>

namespace polygon_coverage_planning {

  bool polygonWithHolesStampedMsgFromNode(
      std::shared_ptr<rclcpp::Node> &node, polygon_coverage_msgs::msg::PolygonWithHolesStamped *msg)
  {
    headerMsgFromNode(node, &(msg->header));
    // 使用 declare_parameter 声明参数
    std::vector<std::string> parameterValue = {
      "x: 0, y: 0, z: 0",
      "x: 0, y: 0, z: 0",
      "x: 0, y: 0, z: 0",
      "x: 0, y: 0, z: 0"
    };
    node->declare_parameter("polygon.hull.points", parameterValue);
    node->declare_parameter("polygon.holes.hole1.points", parameterValue);
    node->declare_parameter("polygon.holes.hole2.points", parameterValue);
    // std::vector<geometry_msgs::msg::Point32> points;
    // if (pointMsgFromNode(node, "polygon.hull.points", points)
    //   && pointMsgFromNode(node, "polygon.holes.hole1.points", points)
    //   && pointMsgFromNode(node, "polygon.holes.hole2.points", points)) {
    //   for (size_t i = 0; i < points.size(); ++i) {
    //     RCLCPP_INFO(node->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, points[i].x, points[i].y, points[i].z);
    //   }
    // }
    
    
    std::vector<geometry_msgs::msg::Polygon> polygons; // 创建一个存储 Polygon 对象的向量
    std::vector<geometry_msgs::msg::Point32> points;
    geometry_msgs::msg::Polygon p0; 
    geometry_msgs::msg::Polygon p1;                     // 创建一个 Polygon 对象并设置其内容
    geometry_msgs::msg::Polygon p2; 

    pointMsgFromNode(node, "polygon.hull.points", points);
    p0.points = points;
    msg->polygon.hull = p0;
    // RCLCPP_INFO(node->get_logger(), "polygon.hull.points.size:%d",msg->polygon.hull.points.size());
    // for (size_t i = 0; i < msg->polygon.hull.points.size(); ++i)
    // {
    //   RCLCPP_INFO(node->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i
    //   , msg->polygon.hull.points[i].x
    //   , msg->polygon.hull.points[i].y
    //   , msg->polygon.hull.points[i].z);
    // }
    points.clear();
    
    pointMsgFromNode(node, "polygon.holes.hole1.points", points);
    p1.points = points;
    // 将 Polygon 对象添加到向量中
    polygons.push_back(p1);
    // 将整个向量分配给 msg.holes
    msg->polygon.holes = polygons;

    // RCLCPP_INFO(node->get_logger(), "polygon.holes.hole1.points.size:%d",msg->polygon.holes[0].points.size());
    // for (size_t i = 0; i < msg->polygon.holes[0].points.size(); ++i)
    // {
    //   RCLCPP_INFO(node->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i
    //   , msg->polygon.holes[0].points[i].x
    //   , msg->polygon.holes[0].points[i].y
    //   , msg->polygon.holes[0].points[i].z);
    // }

    points.clear();
    pointMsgFromNode(node, "polygon.holes.hole2.points", points);
    p2.points = points;
    // 将 Polygon 对象添加到向量中
    polygons.push_back(p2);
    // 将整个向量分配给 msg.holes
    msg->polygon.holes = polygons;
    // RCLCPP_INFO(node->get_logger(), "polygon.holes.hole0.points.size:%d",msg->polygon.holes[0].points.size());
    // for (size_t i = 0; i < msg->polygon.holes[0].points.size(); ++i)
    // {
    //   RCLCPP_INFO(node->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i
    //   , msg->polygon.holes[0].points[i].x
    //   , msg->polygon.holes[0].points[i].y
    //   , msg->polygon.holes[0].points[i].z);
    // }
    // RCLCPP_INFO(node->get_logger(), "polygon.holes.hole1.points.size:%d",msg->polygon.holes[1].points.size());
    // for (size_t i = 0; i < msg->polygon.holes[1].points.size(); ++i)
    // {
    //   RCLCPP_INFO(node->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i
    //   , msg->polygon.holes[1].points[i].x
    //   , msg->polygon.holes[1].points[i].y
    //   , msg->polygon.holes[1].points[i].z);
    // }

  }

  bool pointMsgFromNode(
      std::shared_ptr<rclcpp::Node> &node, const std::string& paramName,
      std::vector<geometry_msgs::msg::Point32>& points) {
  std::vector<std::string> retrievedValue;
  if (node->get_parameter(paramName, retrievedValue)) {
    // RCLCPP_INFO(node->get_logger(), "%s:", paramName.c_str());
    for (const std::string& pointStr : retrievedValue) {
      geometry_msgs::msg::Point32 point;

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
      // RCLCPP_INFO(node->get_logger(), "x=%f, y=%f, z=%f", point.x, point.y, point.z);
      points.push_back(point);
      // Now you can work with the geometry_msgs::msg::Point32
    }
    return true;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to get the parameter.");
    return false;
  }
}

bool headerMsgFromNode(std::shared_ptr<rclcpp::Node>& node, std_msgs::msg::Header* msg)
 {
  const std::string kFrameIdKey = "polygon.header.frame_id";
  msg->stamp = builtin_interfaces::msg::Time(); 
  node->declare_parameter<std::string>("polygon.header.frame_id", "world111");
  node->get_parameter("polygon.header.frame_id", msg->frame_id);
  RCLCPP_INFO(node->get_logger(), "frame_id=%s", (msg->frame_id).c_str());
  return true;
}

// bool polygonMsgFromNode( std::shared_ptr<rclcpp::Node>& node,
//                           geometry_msgs::msg::Polygon* msg) {

//   msg->points = std::vector<geometry_msgs::msg::Point32>();
//   node->declare_parameter(kPointsKey,msg->points);

//   try {
//     // Get polygon vertices.
//     node->get_parameter(kPointsKey, msg->points);
//     std::vector<double> double_array = parameter.as_double_array();

//     // Resize the message and fill it with Point32 elements.
//     msg->points.resize(double_array.size() / 3); // Assuming each point has x, y, z coordinates.
//     int point_idx = 0;
//     for (size_t i = 0; i < double_array.size(); i += 3)
//     {
//       geometry_msgs::msg::Point32 p;
//       p.x = double_array[i];
//       p.y = double_array[i + 1];
//       p.z = double_array[i + 2];
//       msg->points[point_idx++] = p;
//     }
    
//   } catch (const std::exception& e) {
//     RCLCPP_ERROR_STREAM(node->get_logger(),"Node client threw error: " << e.what());
//     return false;  
//   }
//   return true;
// }

// bool polygonWithHolesMsgFromNode(
//     std::shared_ptr<rclcpp::Node>& node,
//     polygon_coverage_msgs::msg::PolygonWithHoles msg) {

//   const std::string kHullKey = "hull";
//   const std::string kHolesKey = "holes";

//   try {
//     // Get hull.
//     if (!polygonMsgFromNode(node, msg.hull)) {
//       RCLCPP_WARN(node->get_logger(),"Missing %s key or failed loading polygon. Resetting to default", kHullKey.c_str());
//       msg.hull = geometry_msgs::msg::Polygon();
//     }

//     // Get holes.
//     rclcpp::Parameter parameter;
//     node->get_parameter(kHolesKey, parameter);
//     std::vector<double> double_array = parameter.as_double_array();
//     msg->holes.resize(double_array.size());
//     for (size_t i = 0; i < double_array.size(); ++i) {
//       geometry_msgs::msg::Polygon p;
//       if (!polygonMsgFromNode(node, &p)) {
//         RCLCPP_WARN(node->get_logger(),
//                     "Loading hole %zu failed. Resetting to default.", i);
//         p = geometry_msgs::msg::Polygon();
//       }
//       msg->holes[i] = p;
//     }
//   } catch (const std::exception& e) {
//     RCLCPP_ERROR_STREAM(node->get_logger(),"node client threw error: " << e.what());
//     return false;  
//   return true;
//   }
// }

}  // namespace polygon_coverage_planning
