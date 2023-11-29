cmake_minimum_required(VERSION 3.8)
project(polygon_coverage_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(polygon_coverage_geometry REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_files
  "msg/PolygonWithHoles.msg"
  "msg/PolygonWithHolesStamped.msg"
  "srv/PlannerService.srv"
  "srv/PolygonService.srv"
  DEPENDENCIES geometry_msgs std_msgs trajectory_msgs
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/msg_from_xml_rpc.cc
  src/conversion.cc
)
target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${rclcpp_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${trajectory_msgs_INCLUDE_DIRS}
    ${builtin_interfaces_INCLUDE_DIRS}
    ${polygon_coverage_geometry_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}/rosidl_generator_cpp

)
# rosidl_generate_interfaces(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

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
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )
#导出include
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
