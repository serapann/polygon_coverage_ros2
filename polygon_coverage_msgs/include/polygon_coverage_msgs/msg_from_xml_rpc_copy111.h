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

#ifndef POLYGON_COVERAGE_MSGS_MSG_FROM_XML_RPC_H_
#define POLYGON_COVERAGE_MSGS_MSG_FROM_XML_RPC_H_
#include <string>

#include <geometry_msgs/msg/point32.hpp>
// #include <RCLCPP/assert.h>
// #include <RCLCPP/console.h>
#include <rclcpp/rclcpp.hpp> 
#include <cassert>
// #include <std_msgs/Header.h>
// #include <std_msgs/Time.h>
#include <builtin_interfaces/msg/time.hpp>
// #include <xmlrpcpp/XmlRpc.h>

// #include <polygon_coverage_msgs/PolygonWithHoles.h>
// #include <polygon_coverage_msgs/PolygonWithHolesStamped.h>
#include <polygon_coverage_msgs/msg/polygon_with_holes.hpp>
#include <polygon_coverage_msgs/msg/polygon_with_holes_stamped.hpp>

namespace polygon_coverage_planning {

// Check if a XML RPC member exists and return an error message if not.
// bool hasMember(const XmlRpc::XmlRpcValue& xml_rpc, const std::string& key);
bool hasParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& key);

// Check if a XML RPC value has the correct type and return an error message if
// not.
// bool checkType(XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
//                const XmlRpc::XmlRpcValue::Type& expected_type);
bool checkParameterType(const std::shared_ptr<rclcpp::Node>& node, 
                  const std::string& key, rclcpp::ParameterType expected_type);
/*
// Read an elementary type from an XML RPC.
template <typename T>
inline bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, T* result) {
  assert(result);

  const bool has_member = hasMember(xml_rpc, key);
  const bool is_correct_type = checkType(xml_rpc, key, expected_type);
  rclcpp::Logger logger = get_logger("readElementaryTypeFromXmlRpc"); 
  if (has_member && is_correct_type) {
    if (expected_type == XmlRpc::XmlRpcValue::TypeBoolean ||
        expected_type == XmlRpc::XmlRpcValue::TypeInt ||
        expected_type == XmlRpc::XmlRpcValue::TypeDouble ||
        expected_type == XmlRpc::XmlRpcValue::TypeString) {
      // Catch special cases.
      *result = static_cast<T>(xml_rpc[key]);
      return true;
    } else {
      RCLCPP_ERROR_STREAM(logger,"Method not implemented for type " << expected_type
                                                          << ".");
      return false;
    }
  } else {
    RCLCPP_WARN_STREAM(logger,"Failed reading elementary XML RPC member.");
    RCLCPP_WARN_STREAM_COND(!has_member,logger,
                         "Member with key " << key << " does not exist.");
    RCLCPP_WARN_STREAM_COND(!is_correct_type,logger,
                         "Member with key " << key
                                            << " does not have expected type "
                                            << expected_type << ".");
    return false;  // XML RPC lacks member or member has wrong data type.
  }
}*/
template <typename T>
inline bool readElementaryTypeFromParameter(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& key,
    rclcpp::ParameterType expected_type, T* result) {
  assert(result);

  if (node->has_parameter(key)) {
    rclcpp::Parameter parameter;
    if (node->get_parameter(key, parameter)) {
      if (parameter.get_type() == expected_type) {
        if (expected_type == rclcpp::ParameterType::PARAMETER_BOOL ||
            expected_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
            expected_type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
            expected_type == rclcpp::ParameterType::PARAMETER_STRING) {
          // 处理特殊情况。
          *result = parameter.get_value<T>();
          return true;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Method not implemented for type %d.", static_cast<int>(expected_type));
          return false;
        }
      } else {
        RCLCPP_WARN(node->get_logger(), "Failed reading elementary parameter.");
        RCLCPP_WARN(node->get_logger(), "Parameter has unexpected type %d (expected %d).", static_cast<int>(parameter.get_type()), static_cast<int>(expected_type));
        return false;  // 参数类型不匹配。
      }
    } else {
      RCLCPP_WARN(node->get_logger(), "Failed to get parameter.");
      return false;  // 获取参数失败。
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "Parameter not found: %s", key.c_str());
    return false;  // 参数不存在。
  }
}

// // Catch special cases that cannot be casted.
// // Unsigned int.
// bool readElementaryTypeFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
//     const XmlRpc::XmlRpcValue::Type& expected_type, unsigned int* result);
bool readElementaryTypeFromParameter(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& key,
    rclcpp::ParameterType expected_type, unsigned int* result);
// // Float.
// bool readElementaryTypeFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
//     const XmlRpc::XmlRpcValue::Type& expected_type, float* result);
bool readElementaryTypeFromParameter(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& key,
    rclcpp::ParameterType expected_type, float* result);
    
// // Stamp from XML RPC.
// bool timeFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, ROS::Time* t);
bool timeFromNode(const std::shared_ptr<rclcpp::Node>& node, builtin_interfaces::msg::Time& t);

// // Header from XML RPC.
// bool headerMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, std_msgs::Header* msg);
bool headerMsgFromNode(const std::shared_ptr<rclcpp::Node>& node, std_msgs::msg::Header& msg);

// // Point32 from XML RPC.
// bool point32MsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
//                           geometry_msgs::msg::Point32* msg);
bool point32MsgFromNode(const std::shared_ptr<rclcpp::Node>& node,
                          geometry_msgs::msg::Point32* msg);

// // Polygon from XML RPC.
// bool polygonMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
//                           geometry_msgs::msg::Polygon* msg);
bool polygonMsgFromNode(const std::shared_ptr<rclcpp::Node>& node,
                          geometry_msgs::msg::Polygon* msg);

// // PolygonWithHoles from XML RPC.
// bool polygonWithHolesMsgFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc, polygon_coverage_msgs::msg::PolygonWithHoles* msg);
bool polygonWithHolesMsgFromNode(
    std::shared_ptr<rclcpp::Node> & node, polygon_coverage_msgs::msg::PolygonWithHoles *msg);

// // PolygonWithHolesStamped from XML RPC.
// bool PolygonWithHolesStampedMsgFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc,
//     polygon_coverage_msgs::msg::PolygonWithHolesStamped* msg);
bool PolygonWithHolesStampedMsgFromNode(
    const std::shared_ptr<rclcpp::Node>& node,
    polygon_coverage_msgs::msg::PolygonWithHolesStamped* msg);

}  // namespace polygon_coverage_planning

bool pointMsgFromNode(
      std::shared_ptr<rclcpp::Node>  &node, const std::string& paramName,
      std::vector<geometry_msgs::msg::Point32>& points);

#endif  // POLYGON_COVERAGE_MSGS_MSG_FROM_XML_RPC_H_
