cmake_minimum_required(VERSION 3.8)
project(polygon_coverage_ros2)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
#add _auto
find_package(rclcpp REQUIRED)
find_library(CGAL_LIBRARY
             NAMES cgal
            #  PATHS /usr/lib/x86_64-linux-gnu/cmake/CGAL)
             PATHS /home/sera/CGAL-5.0.2/install/CGAL/lib/cmake/CGAL)
#             PATHS /usr/local/lib/cmake/CGAL)
#Linking third-party libraries
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(polygon_coverage_msgs REQUIRED)
find_package(polygon_coverage_geometry REQUIRED)
find_package(polygon_coverage_planners REQUIRED)
find_package(polygon_coverage_solvers REQUIRED)
find_package(rviz_polygon_tool REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()
#Used with ament_cmake_auto to automatically generate dependencies

############
# YAML-CPP #
############
# Link against system catkin yaml-cpp if installed.
find_package(PkgConfig)
find_package(yaml_cpp_vendor REQUIRED)
# find_package(yaml_cpp_catkin QUIET)
# if(${yaml_cpp_catkin_FOUND})
#     message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
#     set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
#     set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
# else()
#     message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
#     pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
# endif()

# Download test instances.
include(ExternalProject)
ExternalProject_Add(
  pwh_instances
  URL https://polybox.ethz.ch/index.php/s/7J5HPRR6lM22TnK/download
  DOWNLOAD_NAME pwh.tar.gz
  URL_MD5 87b964a974f6a8dd174dfe393b631a61
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

#############
# LIBRARIES #
############# 
# add_library(${PROJECT_NAME}_patches SHARED
#   src/polygon_planner_base.cc
#   src/ros_interface.cc
# )
add_library(${PROJECT_NAME} SHARED
  src/polygon_planner_base.cc
  src/ros_interface.cc
  src/coverage_planner.cc
  # src/shortest_path_planner.cc
)
target_link_libraries(${PROJECT_NAME} 
# ${PROJECT_NAME}_patches
${CGAL_LIBRARIES}
${CGAL_3RD_PARTY_LIBRARIES}
${rclcpp_LIBRARIES}
${rcl_interfaces_LIBRARIES}
${nav_msgs_LIBRARIES}
${geometry_msgs_LIBRARIES}
${visualization_msgs_LIBRARIES}
${trajectory_msgs_LIBRARIES}
${std_srvs_LIBRARIES}
${tf2_eigen_LIBRARIES}
${builtin_interfaces_LIBRARIES}
${polygon_coverage_geometry_LIBRARIES}
${polygon_coverage_msgs_LIBRARIES}
${polygon_coverage_planners_LIBRARIES}
${polygon_coverage_solvers_LIBRARIES}
${rviz_polygon_tool_LIBRARIES}
-lgmp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${CGAL_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${trajectory_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${builtin_interfaces_INCLUDE_DIRS}
    ${polygon_coverage_geometry_INCLUDE_DIRS}
    ${polygon_coverage_msgs_INCLUDE_DIRS}
    ${polygon_coverage_planners_INCLUDE_DIRS}
    ${polygon_coverage_solvers_INCLUDE_DIRS}
    ${rviz_polygon_tool_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

############
# Binaries #
############
add_executable(coverage_planner
  src/coverage_planner_node.cc
)
target_link_libraries(coverage_planner ${PROJECT_NAME})

# add_executable(coverage_planner_exact
#   src/coverage_planner_exact_node.cc
# )
# target_link_libraries(coverage_planner_exact ${PROJECT_NAME})

# add_executable(coverage_planner_exact_preprocessed
#   src/coverage_planner_exact_preprocessed_node.cc
# )
# target_link_libraries(coverage_planner_exact_preprocessed ${PROJECT_NAME})

# add_executable(shortest_path_planner
#   src/shortest_path_planner_node.cc
# )
# target_link_libraries(shortest_path_planner ${PROJECT_NAME})

# add_executable(sweep_test
#   test/sweep_test_node.cc
# )
# target_link_libraries(sweep_test ${PROJECT_NAME})

#安装库文件
install(TARGETS ${PROJECT_NAME}
        # ARCHIVE DESTINATION lib
        # LIBRARY DESTINATION lib/${PROJECT_NAME}????????????
        LIBRARY DESTINATION lib 
        # RUNTIME DESTINATION bin
)
install(TARGETS 
  coverage_planner 
  # coverage_planner_exact 
  # coverage_planner_exact_preprocessed 
  # shortest_path_planner sweep_test
  RUNTIME DESTINATION  lib/${PROJECT_NAME}
)
install(
  DIRECTORY
   launch/ 
  DESTINATION
    share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY
   images/ 
  DESTINATION
    share/${PROJECT_NAME}/images
)
install(
  DIRECTORY
   cfg/ 
  DESTINATION
    share/${PROJECT_NAME}/cfg
)


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
