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

#include "polygon_coverage_planners/planners/polygon_stripmap_planner_exact.h"

// #include <ros/assert.h>
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <cassert>

namespace polygon_coverage_planning {

bool PolygonStripmapPlannerExact::setupSolver() {
  RCLCPP_INFO(rclcpp::get_logger("PolygonStripmapPlannerExact::setupSolver"),"Creating boolean lattice.");
  boolean_lattice_ = boolean_lattice::BooleanLattice(
      sweep_plan_graph_.getDecomposition().size());
  if (!boolean_lattice_.isInitialized()) {
    RCLCPP_ERROR(rclcpp::get_logger("PolygonStripmapPlannerExact::setupSolver"),"Cannot create boolean lattice.");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("PolygonStripmapPlannerExact::setupSolver"),"Initializing product graph.");
  gtspp_product_graph_ = gtspp_product_graph::GtsppProductGraph(
      &sweep_plan_graph_, &boolean_lattice_);

  return preprocess();
}

bool PolygonStripmapPlannerExact::preprocess() {
  RCLCPP_INFO(rclcpp::get_logger("PolygonStripmapPlannerExact::preprocess"),"Preset product graph.");
  if (!gtspp_product_graph_.createOnline()) {
    RCLCPP_ERROR(rclcpp::get_logger("PolygonStripmapPlannerExact::preprocess"),"Could not create product graph.");
    return false;
  }
  return true;
}

bool PolygonStripmapPlannerExact::runSolver(
    const Point_2& start, const Point_2& goal,
    std::vector<Point_2>* solution) const {
  assert(solution);

  RCLCPP_INFO(rclcpp::get_logger("PolygonStripmapPlannerExact::runSolver"),"Start solving GTSP using exact solver without preprocessing.");
  return gtspp_product_graph_.solveOnline(start, goal, solution);
}

}  // namespace polygon_coverage_planning
