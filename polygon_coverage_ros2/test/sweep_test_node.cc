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

#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/sweep.h>
#include <polygon_coverage_msgs/msg/polygon_with_holes_stamped.hpp>
#include <polygon_coverage_msgs/msg_from_xml_rpc.h>
#include <polygon_coverage_ros2/ros_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <limits>

using namespace polygon_coverage_planning;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto nh_private = std::make_shared<rclcpp::Node>("sweep_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  rclcpp::Logger logger = rclcpp::get_logger("sweep_test");
  // Load polygon.
  // XmlRpc::XmlRpcValue polygon_xml_rpc;
  const std::shared_ptr<rclcpp::Node>& polygon_node = std::make_shared<rclcpp::Node>("polygon_node");
  rclcpp::Parameter polygon_parameter;
  const std::string polygon_param_name = "polygon";
  if (!nh_private->get_parameter(polygon_param_name, polygon_parameter)) {
    RCLCPP_ERROR(logger,"No polygon set on parameter server.");
    return 0;
  }
  polygon_coverage_msgs::msg::PolygonWithHolesStamped poly_msg;
  if (!PolygonWithHolesStampedMsgFromNode(polygon_node, &poly_msg)) {
    RCLCPP_ERROR(logger,"Failed to get polygon from XMLRpc.");
    return 0;
  }
  PolygonWithHoles poly;
  double altitude;
  std::string global_frame_id;
  if (polygonFromMsg(poly_msg, &poly, &altitude, &global_frame_id)) {
    RCLCPP_INFO_STREAM(logger,"Successfully loaded polygon.");
    RCLCPP_INFO_STREAM(logger,"Altiude: " << altitude << "m");
    RCLCPP_INFO_STREAM(logger,"Global frame: " << global_frame_id);
    RCLCPP_INFO_STREAM(logger,"Polygon:" << poly);
  } else {
    RCLCPP_WARN_STREAM(logger,"Failed reading polygon message from parameter server.");
  }
  // Publish polygon.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub 
  = nh_private->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization", 1);
  visualization_msgs::msg::MarkerArray polygon_markers;
  const double kPolygonLineSize = 0.8;
  const double kPolygonPointSize = 0.8;
  createPolygonMarkers(poly, altitude, global_frame_id, "polygon",
                       Color::Black(),
                       Color::Black(), kPolygonLineSize,
                       kPolygonPointSize, &polygon_markers);

  // Publish waypoint list.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub 
  = nh_private->create_publisher<visualization_msgs::msg::MarkerArray>(
      "waypoints", 1);                                                          

  // Compute sweep permutations.
  std::vector<std::vector<Point_2>> waypoints;
  const double kMaxSweepDistance = 9.0;
  Polygon_2 hull = poly.outer_boundary();
  computeAllSweeps(hull, kMaxSweepDistance, &waypoints);

  size_t i = 0;
  while (rclcpp::ok()) {
    for (const Point_2& p : waypoints[i]) {
      RCLCPP_INFO_STREAM(logger,"p: " << p);
    }

    // The planned path:
    visualization_msgs::msg::MarkerArray all_markers;
    visualization_msgs::msg::Marker path_points, path_line_strips;
    const double kPathLineSize = 0.4;
    const double kPathPointSize = 0.8;
    createMarkers(waypoints[i], altitude, global_frame_id, "vertices_and_strip",
                  Color::Gray(),
                  Color::Gray(), kPathLineSize,
                  kPathPointSize, &path_points, &path_line_strips);
    all_markers.markers.push_back(path_points);
    all_markers.markers.push_back(path_line_strips);

    // Start and end points
    visualization_msgs::msg::Marker start_point, end_point;
    createStartAndEndPointMarkers(waypoints[i].front(), waypoints[i].back(),
                                  altitude, global_frame_id, "points",
                                  &start_point, &end_point);
    all_markers.markers.push_back(start_point);
    all_markers.markers.push_back(end_point);

    // Start and end text.
    visualization_msgs::msg::Marker start_text, end_text;
    createStartAndEndTextMarkers(waypoints[i].front(), waypoints[i].back(),
                                 altitude, global_frame_id, "text", &start_text,
                                 &end_text);
    all_markers.markers.push_back(start_text);
    all_markers.markers.push_back(end_text);

    // Start arrow.
    visualization_msgs::msg::Marker start_arrow;
    start_arrow.type = visualization_msgs::msg::Marker::ARROW;
    start_arrow.action = visualization_msgs::msg::Marker::ADD;
    start_arrow.points.push_back(start_point.pose.position);
    geometry_msgs::msg::Point end;
    end.x = CGAL::to_double(std::next(waypoints[i].begin())->x());
    end.y = CGAL::to_double(std::next(waypoints[i].begin())->y());
    end.z = altitude;
    start_arrow.points.push_back(end);
    start_arrow.color = Color::Red();
    start_arrow.scale.x = 1.6;
    start_arrow.scale.y = 4.0;
    start_arrow.scale.z = 6.0;
    start_arrow.header.frame_id = global_frame_id;
    rclcpp::Time current_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time ros2_time;
    ros2_time.sec = current_time.seconds();
    ros2_time.nanosec = current_time.nanoseconds();
    start_arrow.header.stamp = ros2_time;
    start_arrow.ns = "dir";
    all_markers.markers.push_back(start_arrow);

    all_markers.markers.insert(all_markers.markers.end(),
                               polygon_markers.markers.begin(),
                               polygon_markers.markers.end());
    pub->publish(all_markers);

    i++;
    i = i % waypoints.size();

    RCLCPP_INFO(logger,"Press ENTER to continue...");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  return 0;
}
