cmake_minimum_required(VERSION 3.8)
project(polygon_coverage_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(polygon_coverage_solvers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_library(CGAL_LIBRARY
             NAMES cgal
            #  PATHS /usr/lib/x86_64-linux-gnu/cmake/CGAL)
             PATHS /home/sera/CGAL-5.0.2/install/CGAL/lib/cmake/CGAL)
#             PATHS /usr/local/lib/cmake/CGAL)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} SHARED
  src/bcd.cc
  src/boolean.cc
  src/cgal_comm.cc
  src/decomposition.cc
  src/offset.cc
  src/sweep.cc
  src/tcd.cc
  src/triangulation.cc
  src/visibility_graph.cc
  src/visibility_polygon.cc
  src/weakly_monotone.cc
)
target_link_libraries(${PROJECT_NAME}
 ${CGAL_LIBRARIES} 
 ${CGAL_3RD_PARTY_LIBRARIES}
 ${rclcpp_LIBRARIES}
 -lgmp
 -lmpfr
 )
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${polygon_coverage_solvers_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS} 
  ${CGAL_3RD_PARTY_INCLUDE_DIRS}
  # ${CMAKE_CURRENT_SOURCE_DIR}/src
  
)
# message(${CMAKE_CURRENT_SOURCE_DIR}/src)
#########
# TESTS #
#########
ament_add_gtest(test_bcd
  test/bcd-test.cpp
)
target_link_libraries(test_bcd ${PROJECT_NAME})

ament_add_gtest(test_cgal_comm
  test/cgal_comm-test.cpp
)
target_link_libraries(test_cgal_comm ${PROJECT_NAME})

ament_add_gtest(test_offset
  test/offset-test.cpp
)
target_link_libraries(test_offset ${PROJECT_NAME})

ament_add_gtest(test_sweep
  test/sweep-test.cpp
)
target_link_libraries(test_sweep ${PROJECT_NAME})

ament_add_gtest(test_visibility_graph
  test/visibility_graph-test.cpp
)
target_link_libraries(test_visibility_graph ${PROJECT_NAME})

ament_add_gtest(test_visibility_polygon
  test/visibility_polygon-test.cpp
)
target_link_libraries(test_visibility_polygon ${PROJECT_NAME})

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
