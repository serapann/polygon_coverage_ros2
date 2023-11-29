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

#ifndef RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
#define RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <OGRE/OgreRay.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>

#include <polygon_coverage_geometry/cgal_definitions.h>

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreVector3.h>
// #include <ros/console.h>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cassert>
#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

#include <QKeyEvent>

// #include <sensor_msgs/msg/point_cloud2.hpp>
#include <polygon_coverage_msgs/msg/polygon_with_holes_stamped.hpp>

namespace rviz_polygon_tool {

// Declare polygon tool as subclass of rviz::Tool.
// Left click: Insert a new vertex before the selected vertex.
// Right click: Remove a vertex, select the next vertex.
// 'h': Add hole
// 'n': Select next polygon
// 'v': Select next vertex
// 'r': Reset currently selected polygon
// 'c': Clear all
// Enter: Publish polygon if valid
class PolygonTool : public rviz_common::Tool
{
  Q_OBJECT
// // class PolygonTool : public rviz::Tool {
// //   Q_OBJECT

 public:
  PolygonTool();
  virtual ~PolygonTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  // int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
  // int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;
  int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;

  bool getPointOnPlaneFromWindowXY(rviz_common::RenderPanel* panel, Ogre::Plane& plane, int window_x, int window_y, Ogre::Vector3& intersection_out);

 private:
  // User input.
  // void clickLeft(const rviz::ViewportMouseEvent& event);
  // void clickRight(const rviz::ViewportMouseEvent& event);
  void clickLeft(const rviz_common::ViewportMouseEvent& event);
  void clickRight(const rviz_common::ViewportMouseEvent& event);

  // Action.
  void createVertex(const Ogre::Vector3& position);
  void deleteVertex(const Ogre::Vector3& position);
  void addHole();
  void nextPolygon();
  void nextVertex();
  void resetPolygon();
  void clearAll();
  void publishPolygon();
  void updateStatus();
  void removeEmptyHoles();
  // void increaseAltitude(rviz::ViewportMouseEvent& event);
  // void decreaseAltitude(rviz::ViewportMouseEvent& event);
  void increaseAltitude(rviz_common::ViewportMouseEvent& event);
  void decreaseAltitude(rviz_common::ViewportMouseEvent& event);

  std::list<Polygon_2> polygons_;
  std::list<Polygon_2>::iterator polygon_selection_;  // 0: hull, 1..N-1: holes
  VertexIterator vertex_selection_;
  double altitude_;

  // Rendering
  void renderPolygon(const Polygon_2& polygon, const Ogre::ColourValue& c);
  void renderPolygons();
  Ogre::SceneNode* polygon_node_;

  // Sphere currently displayed.
  Ogre::SceneNode* moving_vertex_node_;
  // rviz::Shape* sphere_;
  rviz_rendering::Shape* sphere_;

  //   // ROS messaging
  //   ros::NodeHandle nh_;
  std::shared_ptr<rclcpp::Node> nh_;
  //   ros::Publisher polygon_pub_;
  rclcpp::Publisher<polygon_coverage_msgs::msg::PolygonWithHolesStamped>::SharedPtr polygon_pub_;
};

}  // namespace rviz_polygon_tool

#endif  // RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
