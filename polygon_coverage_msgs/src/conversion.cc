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

#include "polygon_coverage_msgs/conversion.h"

#include <geometry_msgs/msg/point32.h>
// #include <geometry_msgs/msg/polygon.h>
// #include <ros/assert.h>
#include <cassert>
namespace polygon_coverage_planning {

bool convertPolygonWithHolesToMsg(
    const PolygonWithHoles& pwh, const double altitude,
    polygon_coverage_msgs::msg::PolygonWithHoles* msg) {
  if (!msg) {
    return false;
  }

  convertPolygonToMsg(pwh.outer_boundary(), altitude, &msg->hull);
  for (auto h = pwh.holes_begin(); h != pwh.holes_end(); ++h) {
    geometry_msgs::msg::Polygon hole;
    convertPolygonToMsg(*h, altitude, &hole);
    msg->holes.push_back(hole);
  }
}

bool convertPolygonToMsg(const Polygon_2& p, const double altitude,
                         geometry_msgs::msg::Polygon* msg) {
  if (!msg) {
    return false;
  }
  msg->points.clear();

  for (auto v = p.vertices_begin(); v != p.vertices_end(); ++v) {
    geometry_msgs::msg::Point32 pt;
    pt.x = CGAL::to_double(v->x());
    pt.y = CGAL::to_double(v->y());
    pt.z = altitude;
    msg->points.push_back(pt);
  }
}

}  // namespace polygon_coverage_planning
