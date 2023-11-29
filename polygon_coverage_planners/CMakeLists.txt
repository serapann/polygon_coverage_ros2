cmake_minimum_required(VERSION 3.8)
project(polygon_coverage_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_library(CGAL_LIBRARY
             NAMES cgal
            #  PATHS /usr/lib/x86_64-linux-gnu/cmake/CGAL)
             PATHS /home/sera/CGAL-5.0.2/install/CGAL/lib/cmake/CGAL)
#             PATHS /usr/local/lib/cmake/CGAL)
find_package(polygon_coverage_solvers REQUIRED)
find_package(polygon_coverage_geometry REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_auto_find_build_dependencies()

# Add mono to invoke gk_ma.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
#include_directories(${MONO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/cost_functions/path_cost_functions.cc
  src/graphs/gtspp_product_graph.cc
  src/graphs/sweep_plan_graph.cc
  src/timing.cc
  src/planners/polygon_stripmap_planner.cc
  src/planners/polygon_stripmap_planner_exact.cc
  src/planners/polygon_stripmap_planner_exact_preprocessed.cc
)
target_link_libraries(${PROJECT_NAME} 
  ${polygon_coverage_geometry_LIBRARIES}
  ${polygon_coverage_solvers_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${CGAL_3RD_PARTY_LIBRARIES}
  ${rcutils_LIBRARIES}
  ${rclcpp_LIBRARIES}
  -lgmp
  -lmpfr

  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${polygon_coverage_solvers_INCLUDE_DIRS}
  ${polygon_coverage_geometry_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS} 
  ${CGAL_3RD_PARTY_INCLUDE_DIRS}
  ${MONO_INCLUDE_DIRS}
)

#########
# TESTS #
#########
ament_add_gtest(test_planners
  test/planners-test.cpp
 )
target_link_libraries(test_planners 
${PROJECT_NAME}
${polygon_coverage_geometry_LIBRARIES}
)

##########
# EXPORT #
##########
#安装库文件
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
#安装头文件
#ament_export_include_directories(${CMAKE_INSTALL_PREFIX})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )
#导出include
#ament_export_dependencies(${PROJECT_NAME})
#ament_export_targets(cgal_comm.cc)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
