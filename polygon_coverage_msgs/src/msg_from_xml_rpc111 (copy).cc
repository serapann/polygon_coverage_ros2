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

// Check if a XML RPC member exists and return an error message if not.
// bool hasMember(const XmlRpc::XmlRpcValue& xml_rpc, const std::string& key) {
//   if (xml_rpc.hasMember(key)) {
//     return true;
//   } else {
//     ROS_WARN_STREAM("XML RPC has no key: " << key.c_str());
//     return false;
//   }
// }
bool hasParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& key) {
  if (node->has_parameter(key)) {
    return true;
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("hasParameter")," key: " << key.c_str() << "does not exist.");
    return false;
  }
}

// Check if a XML RPC value has the correct type and return an error message if
// not.
// bool checkType(XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
//                const XmlRpc::XmlRpcValue::Type& expected_type) {
//   if (xml_rpc[key].getType() == expected_type) {
//     return true;
//   } else {
//     ROS_ERROR_STREAM("XML RPC expected type: " << expected_type << " is type: "
//                                                << xml_rpc[key].getType());
//     return false;
//   }
// }
bool checkParameterType(const std::shared_ptr<rclcpp::Node>& node, const std::string& key, rclcpp::ParameterType expected_type) {
    rclcpp::Parameter parameter;
    if (node->get_parameter(key, parameter)) {
        if (parameter.get_type() == expected_type) {
            return true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "Parameter '%s' expected type: %d is type: %d", key.c_str(), static_cast<int>(expected_type), static_cast<int>(parameter.get_type()));
            return false;
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Parameter '%s' does not exist.", key.c_str());
        return false;
    }
}

// // Catch special cases that cannot be casted.
// // Unsigned int.
// bool readElementaryTypeFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
//     const XmlRpc::XmlRpcValue::Type& expected_type, unsigned int* result) {
//   ROS_ASSERT(result);

//   int temp_result = -1;
//   if (readElementaryTypeFromXmlRpc(xml_rpc, key, expected_type, &temp_result)) {
//     *result = static_cast<unsigned int>(temp_result);
//     return true;
//   } else {
//     return false;
//   }
// }
bool readElementaryTypeFromParameter(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& key,
    rclcpp::ParameterType expected_type, unsigned int* result) {
  if (result == nullptr) {
    return false;
  }

  int temp_result = -1;
  if (readElementaryTypeFromParameter(node, key, expected_type, &temp_result)) {
    *result = static_cast<unsigned int>(temp_result);
    return true;
  } else {
    return false;
  }
}

// // Float.
// bool readElementaryTypeFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
//     const XmlRpc::XmlRpcValue::Type& expected_type, float* result) {
//   ROS_ASSERT(result);

//   double temp_result = 0.0;
//   if (readElementaryTypeFromXmlRpc(xml_rpc, key, expected_type, &temp_result)) {
//     *result = static_cast<float>(temp_result);
//     return true;
//   } else {
//     return false;
//   }
// }
bool readElementaryTypeFromParameter(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& key,
    rclcpp::ParameterType expected_type, float* result) {
  if (result == nullptr) {
    return false;
  }

  double temp_result = 0.0;
  if (readElementaryTypeFromParameter(node, key, expected_type, &temp_result)) {
    *result = static_cast<float>(temp_result);
    return true;
  } else {
    return false;
  }
}

// // Stamp from XML RPC.
// bool timeFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, ros::Time* t) {
//   ROS_ASSERT(t);

//   const std::string kSecsKey = "secs";
//   const std::string kNsecsKey = "nsecs";

//   try {
//     // Get seconds.
//     int secs = 0;
//     if (!readElementaryTypeFromXmlRpc(xml_rpc, kSecsKey,
//                                       XmlRpc::XmlRpcValue::TypeInt, &secs)) {
//       ROS_WARN_STREAM("Resetting key " << kSecsKey << " to default.");
//       secs = std_msgs::Int32().data;
//     }
//     // Get nano seconds.
//     int nsecs = 0;
//     if (!readElementaryTypeFromXmlRpc(xml_rpc, kNsecsKey,
//                                       XmlRpc::XmlRpcValue::TypeInt, &nsecs)) {
//       ROS_WARN_STREAM("Resetting key " << kNsecsKey << " to default.");
//       nsecs = std_msgs::Int32().data;
//     }
//     *t = ros::Time(secs, nsecs);
//   } catch (const std::exception& e) {
//     ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
bool timeFromNode(const std::shared_ptr<rclcpp::Node>& node, builtin_interfaces::msg::Time& t) {
  // 断言 t 是否有效
  assert(t.nanosec >= 0 && t.nanosec < 1000000000);

  const std::string kSecsKey = "secs";
  const std::string kNsecsKey = "nsecs";

  try {
    // Get seconds.
    int secs = 0;
    if (!readElementaryTypeFromParameter(node, kSecsKey,
                                         rclcpp::ParameterType::PARAMETER_INTEGER, &secs))
    {
            RCLCPP_WARN(node->get_logger(), "Resetting key %s to default.", kSecsKey.c_str());
            secs = std_msgs::msg::Int32().data;
    }
    // Get nano seconds.
    unsigned int nsecs = 0;
    if (!readElementaryTypeFromParameter(node, kNsecsKey, rclcpp::ParameterType::PARAMETER_INTEGER, &nsecs))
    {
            RCLCPP_WARN(node->get_logger(), "Resetting key %s to default.", kNsecsKey.c_str());
            nsecs = std_msgs::msg::Int32().data;
    }
    t.sec = static_cast<int32_t>(secs);
    t.nanosec = static_cast<uint32_t>(nsecs);
  } catch (const std::exception& e) {
    // ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("node_error"), "node client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// // Header from XML RPC.
// bool headerMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, std_msgs::Header* msg) {
//   ROS_ASSERT(msg);

//   const std::string kSeqKey = "seq";
//   const std::string kStampKey = "stamp";
//   const std::string kFrameIdKey = "frame_id";

//   try {
//     // Get sequence.
//     if (!readElementaryTypeFromXmlRpc(
//             xml_rpc, kSeqKey, XmlRpc::XmlRpcValue::TypeInt, &msg->seq)) {
//       ROS_WARN_STREAM("Resetting key " << kSeqKey << " to default.");
//       msg->seq = std_msgs::UInt32().data;
//     }

//     // Get time stamp.
//     if (!hasMember(xml_rpc, kStampKey) ||
//         !timeFromXmlRpc(xml_rpc[kStampKey], &(msg->stamp))) {
//       ROS_WARN_STREAM("Missing "
//                       << kStampKey
//                       << " key or failed reading time. Resetting to default");
//       msg->stamp = std_msgs::Time().data;
//     }

//     // Get frame id.
//     if (!readElementaryTypeFromXmlRpc(xml_rpc, kFrameIdKey,
//                                       XmlRpc::XmlRpcValue::TypeString,
//                                       &msg->frame_id)) {
//       ROS_WARN_STREAM("Resetting key " << kFrameIdKey << " to default.");
//       msg->frame_id = std_msgs::String().data;
//     }

//   } catch (const std::exception& e) {
//     ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
bool headerMsgFromNode(const std::shared_ptr<rclcpp::Node>& node, std_msgs::msg::Header* msg)
 {
  assert(msg->stamp.sec != 0 || msg->stamp.nanosec != 0);


  const std::string kStampKey = "stamp";
  const std::string kFrameIdKey = "frame_id";

  try {
    // Get time stamp.
    if (!hasParameter(node, kStampKey) ||
        !timeFromNode(node, msg->stamp)) {
      RCLCPP_WARN(node->get_logger(), "Missing %s key or failed reading time. Resetting to default", kStampKey.c_str());
      msg->stamp = builtin_interfaces::msg::Time(); // 设置为默认时间
    }

    // Get frame id.
    if (!readElementaryTypeFromParameter(node, kFrameIdKey,
                                      rclcpp::ParameterType::PARAMETER_STRING,
                                      &msg->frame_id)) {
      RCLCPP_WARN(node->get_logger(), "Resetting key %s to default.", kFrameIdKey.c_str());
      msg->frame_id = std_msgs::msg::Int32().data;
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Node client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// // Point32 from XML RPC.
// bool point32MsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
//                           geometry_msgs::msg::Point32* msg) {
//   ROS_ASSERT(msg);

//   const std::string kXKey = "x";
//   const std::string kYKey = "y";
//   const std::string kZKey = "z";

//   try {
//     // Get x.
//     if (!readElementaryTypeFromXmlRpc(
//             xml_rpc, kXKey, XmlRpc::XmlRpcValue::TypeDouble, &msg->x)) {
//       ROS_WARN_STREAM("Resetting key " << kXKey << " to default.");
//       msg->x = std_msgs::Float64().data;
//     }

//     // Get y.
//     if (!readElementaryTypeFromXmlRpc(
//             xml_rpc, kYKey, XmlRpc::XmlRpcValue::TypeDouble, &msg->y)) {
//       ROS_WARN_STREAM("Resetting key " << kYKey << " to default.");
//       msg->y = std_msgs::Float64().data;
//     }

//     // Get z.
//     if (!readElementaryTypeFromXmlRpc(
//             xml_rpc, kZKey, XmlRpc::XmlRpcValue::TypeDouble, &msg->z)) {
//       ROS_WARN_STREAM("Resetting key " << kZKey << " to default.");
//       msg->z = std_msgs::Float64().data;
//     }
//   } catch (const std::exception& e) {
//     ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
bool point32MsgFromNode(const std::shared_ptr<rclcpp::Node>& node,
                        geometry_msgs::msg::Point32* msg) {
  if (!msg) {
    return false; // Check if msg is nullptr.
  }

  const std::string kXKey = "x";
  const std::string kYKey = "y";
  const std::string kZKey = "z";

  try {
    // Get x.
    if (!readElementaryTypeFromParameter(
            node, kXKey, rclcpp::ParameterType::PARAMETER_DOUBLE, &msg->x)) {
      RCLCPP_WARN(node->get_logger(), "Resetting key %s to default.", kXKey.c_str());
      msg->x = std_msgs::msg::Float64().data;
    }

    // Get y.
    if (!readElementaryTypeFromParameter(
            node, kYKey, rclcpp::ParameterType::PARAMETER_DOUBLE, &msg->y)) {
      RCLCPP_WARN(node->get_logger(), "Resetting key %s to default.", kYKey.c_str());
      msg->y = std_msgs::msg::Float64().data;
    }

    // Get z.
    if (!readElementaryTypeFromParameter(
            node, kZKey, rclcpp::ParameterType::PARAMETER_DOUBLE, &msg->z)) {
      RCLCPP_WARN(node->get_logger(), "Resetting key %s to default.", kZKey.c_str());
      msg->z = std_msgs::msg::Float64().data;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Node client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// // Polygon from XML RPC.
// bool polygonMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
//                           geometry_msgs::msg::Polygon* msg) {
//   ROS_ASSERT(msg);

//   const std::string kPointsKey = "points";

//   try {
//     // Get polygon vertices.
//     if (hasMember(xml_rpc, kPointsKey) &&
//         checkType(xml_rpc, kPointsKey, XmlRpc::XmlRpcValue::TypeArray)) {
//       msg->points.resize(xml_rpc[kPointsKey].size());
//       for (int i = 0; i < xml_rpc[kPointsKey].size(); ++i) {
//         geometry_msgs::msg::Point32 p;
//         if (!point32MsgFromXmlRpc(xml_rpc[kPointsKey][i], &p)) {
//           ROS_WARN_STREAM("Loading point " << i
//                                            << " failed. Resetting to default.");
//           p = geometry_msgs::msg::Point32();
//         }
//         msg->points[i] = p;
//       }
//     } else {
//       ROS_WARN_STREAM("Missing or non-array "
//                       << kPointsKey << " member. Resetting to default");
//       msg->points = std::vector<geometry_msgs::msg::Point32>();
//     }
//   } catch (const std::exception& e) {
//     ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
// bool polygonMsgFromNode(const std::shared_ptr<rclcpp::Node>& node,
//                           geometry_msgs::msg::Polygon* msg) {
//   if (!msg || !node) {
//     return false;
//   }

//   const std::string kPointsKey = "points";

//   try {
//     // Get polygon vertices.
//     if (hasParameter(node, kPointsKey) &&
//         checkParameterType(node, kPointsKey, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY))
//     {
//       rclcpp::Parameter parameter;
//       node->get_parameter(kPointsKey, parameter);
//       std::vector<double> double_array = parameter.as_double_array();

//       // Resize the message and fill it with Point32 elements.
//       msg->points.resize(double_array.size() / 3); // Assuming each point has x, y, z coordinates.
//       int point_idx = 0;
//       for (size_t i = 0; i < double_array.size(); i += 3)
//       {
//         geometry_msgs::msg::Point32 p;
//         p.x = double_array[i];
//         p.y = double_array[i + 1];
//         p.z = double_array[i + 2];
//         msg->points[point_idx++] = p;
//       }
//     }
//     else
//     {
//       RCLCPP_WARN(node->get_logger(),
//                   "Missing or non-array parameter %s. Resetting to default", kPointsKey.c_str());
//       msg->points = std::vector<geometry_msgs::msg::Point32>();
//     }
//   } catch (const std::exception& e) {
//     RCLCPP_ERROR_STREAM(node->get_logger(),"Node client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
bool polygonMsgFromNode(const std::shared_ptr<rclcpp::Node>& node,
                          geometry_msgs::msg::Polygon* msg) {
  if (!msg || !node) {
    return false;
  }

  const std::string kPointsKey = "points";
  msg->points = std::vector<geometry_msgs::msg::Point32>();
  node->declare_parameter(kPointsKey,msg->points);

  try {
    // Get polygon vertices.
    node->get_parameter(kPointsKey, msg->points);
    std::vector<double> double_array = parameter.as_double_array();

    // Resize the message and fill it with Point32 elements.
    msg->points.resize(double_array.size() / 3); // Assuming each point has x, y, z coordinates.
    int point_idx = 0;
    for (size_t i = 0; i < double_array.size(); i += 3)
    {
      geometry_msgs::msg::Point32 p;
      p.x = double_array[i];
      p.y = double_array[i + 1];
      p.z = double_array[i + 2];
      msg->points[point_idx++] = p;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Node client threw error: " << e.what());
    return false;  
  }
  return true;
}

// // PolygonWithHoles from XML RPC.
// bool polygonWithHolesMsgFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc,
//     polygon_coverage_msgs::msg::PolygonWithHoles* msg) {
//   ROS_ASSERT(msg);

//   const std::string kHullKey = "hull";
//   const std::string kHolesKey = "holes";

//   try {
//     // Get hull.
//     if (!hasMember(xml_rpc, kHullKey) ||
//         !polygonMsgFromXmlRpc(xml_rpc[kHullKey], &msg->hull)) {
//       ROS_WARN_STREAM(
//           "Missing " << kHullKey
//                      << " key or failed loading polygon. Resetting to default");
//       msg->hull = geometry_msgs::msg::Polygon();
//     }

//     // Get holes.
//     if (hasMember(xml_rpc, kHolesKey) &&
//         checkType(xml_rpc, kHolesKey, XmlRpc::XmlRpcValue::TypeArray)) {
//       msg->holes.resize(xml_rpc[kHolesKey].size());
//       for (int i = 0; i < xml_rpc[kHolesKey].size(); ++i) {
//         geometry_msgs::msg::Polygon p;
//         if (!polygonMsgFromXmlRpc(xml_rpc[kHolesKey][i], &p)) {
//           ROS_WARN_STREAM("Loading hole " << i
//                                           << " failed. Resetting to default.");
//           p = geometry_msgs::msg::Polygon();
//         }
//         msg->holes[i] = p;
//       }
//     } else {
//       ROS_WARN_STREAM("Missing or non-array "
//                       << kHolesKey << " member. Resetting to default");
//       msg->holes = std::vector<geometry_msgs::msg::Polygon>();
//     }
//   } catch (const std::exception& e) {
//     ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
// bool polygonWithHolesMsgFromNode(
//     const std::shared_ptr<rclcpp::Node>& node,
//     polygon_coverage_msgs::msg::PolygonWithHoles* msg) {
//   if (!msg || !node) {
//     return false;
//   }

//   const std::string kHullKey = "hull";
//   const std::string kHolesKey = "holes";

//   try {
//     // Get hull.
//     if (!hasParameter(node, kHullKey) ||
//         !polygonMsgFromNode(node, &msg->hull)) {
//       RCLCPP_WARN(node->get_logger(),
//                   "Missing %s key or failed loading polygon. Resetting to default", kHullKey.c_str());
//       msg->hull = geometry_msgs::msg::Polygon();
//     }

//     // Get holes.
//     if (hasParameter(node, kHolesKey) &&
//         checkParameterType(node, kHolesKey, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)) {
//       rclcpp::Parameter parameter;
//       node->get_parameter(kHolesKey, parameter);
//       std::vector<double> double_array = parameter.as_double_array();

//       msg->holes.resize(double_array.size());
//       for (size_t i = 0; i < double_array.size(); ++i) {
//         geometry_msgs::msg::Polygon p;
//         if (!polygonMsgFromNode(node, &p)) {
//           RCLCPP_WARN(node->get_logger(),
//                       "Loading hole %zu failed. Resetting to default.", i);
//           p = geometry_msgs::msg::Polygon();
//         }
//         msg->holes[i] = p;
//       }
//     }  else {
//       RCLCPP_WARN_STREAM(node->get_logger(),"Missing or non-array "
//                       << kHolesKey << " member. Resetting to default");
//       msg->holes = std::vector<geometry_msgs::msg::Polygon>();
//     }
//   } catch (const std::exception& e) {
//     RCLCPP_ERROR_STREAM(node->get_logger(),"XML RPC client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
bool polygonWithHolesMsgFromNode(
    const std::shared_ptr<rclcpp::Node>& node,
    polygon_coverage_msgs::msg::PolygonWithHoles* msg) {
  if (!msg || !node) {
    return false;
  }

  const std::string kHullKey = "hull";
  const std::string kHolesKey = "holes";

  try {
    // Get hull.
    if (!polygonMsgFromNode(node, &msg->hull)) {
      RCLCPP_WARN(node->get_logger(),"Missing %s key or failed loading polygon. Resetting to default", kHullKey.c_str());
      msg->hull = geometry_msgs::msg::Polygon();
    }

    // Get holes.
    rclcpp::Parameter parameter;
    node->get_parameter(kHolesKey, parameter);
    std::vector<double> double_array = parameter.as_double_array();
    msg->holes.resize(double_array.size());
    for (size_t i = 0; i < double_array.size(); ++i) {
      geometry_msgs::msg::Polygon p;
      if (!polygonMsgFromNode(node, &p)) {
        RCLCPP_WARN(node->get_logger(),
                    "Loading hole %zu failed. Resetting to default.", i);
        p = geometry_msgs::msg::Polygon();
      }
      msg->holes[i] = p;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),"node client threw error: " << e.what());
    return false;  
  return true;
  }
}
// // PolygonWithHolesStamped from XML RPC.
// bool PolygonWithHolesStampedMsgFromXmlRpc(
//     XmlRpc::XmlRpcValue& xml_rpc,
//     polygon_coverage_msgs::msg::PolygonWithHolesStamped* msg) {
//   ROS_ASSERT(msg);

//   const std::string kHeaderKey = "header";
//   const std::string kPolygonWithHolesKey = "polygon";

//   try {
//     // Get header.
//     if (!hasMember(xml_rpc, kHeaderKey) ||
//         !headerMsgFromXmlRpc(xml_rpc[kHeaderKey], &msg->header)) {
//       ROS_WARN_STREAM(
//           "Missing "
//           << kHeaderKey
//           << " member or loading header failed. Resetting to default");
//       msg->header = std_msgs::Header();
//     }

//     // Get polygon with holes.
//     if (!hasMember(xml_rpc, kPolygonWithHolesKey) ||
//         !polygonWithHolesMsgFromXmlRpc(xml_rpc[kPolygonWithHolesKey],
//                                        &msg->polygon)) {
//       ROS_WARN_STREAM("Missing " << kPolygonWithHolesKey
//                                  << " member or loading polygon with holes "
//                                     "failed. Resetting to default");
//       msg->polygon = polygon_coverage_msgs::msg::PolygonWithHoles();
//     }

//   } catch (const std::exception& e) {
//     ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
//     return false;  // XML RPC client exception.
//   }
//   return true;
// }
bool PolygonWithHolesStampedMsgFromNode(
    const std::shared_ptr<rclcpp::Node>& node,
    polygon_coverage_msgs::msg::PolygonWithHolesStamped* msg) {
  if (!msg || !node) {
    return false;
  }

  const std::string kHeaderKey = "header";
  const std::string kPolygonWithHolesKey = "polygon";

  try {
    // Get header.
    if (!hasParameter(node, kHeaderKey) ||
        !headerMsgFromNode(node, &msg->header)) {
      RCLCPP_WARN(node->get_logger(),
                  "Missing %s member or loading header failed. Resetting to default", kHeaderKey.c_str());
      msg->header = std_msgs::msg::Header();
    }

    // Get polygon with holes.
    if (!hasParameter(node, kPolygonWithHolesKey) ||
        !polygonWithHolesMsgFromNode(node, &msg->polygon)) {
      RCLCPP_WARN(node->get_logger(),
                  "Missing %s member or loading polygon with holes failed. Resetting to default", kPolygonWithHolesKey.c_str());
      msg->polygon = polygon_coverage_msgs::msg::PolygonWithHoles();
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),
                 "XML RPC client threw error: %s", e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}


}  // namespace polygon_coverage_planning
