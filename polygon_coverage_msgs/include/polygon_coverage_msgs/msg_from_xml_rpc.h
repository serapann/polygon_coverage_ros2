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

bool polygonWithHolesStampedMsgFromNode(
    std::shared_ptr<rclcpp::Node> & node, polygon_coverage_msgs::msg::PolygonWithHolesStamped *msg);

bool PolygonWithHolesStampedMsgFromNode(
    std::shared_ptr<rclcpp::Node> & node,
    polygon_coverage_msgs::msg::PolygonWithHolesStamped* msg);
bool pointMsgFromNode(
      std::shared_ptr<rclcpp::Node>  &node, const std::string& paramName,
      std::vector<geometry_msgs::msg::Point32>& points);
bool headerMsgFromNode(std::shared_ptr<rclcpp::Node>& node, std_msgs::msg::Header* msg);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_MSGS_MSG_FROM_XML_RPC_H_
