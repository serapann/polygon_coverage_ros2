cmake_minimum_required(VERSION 3.8)
project(rviz_polygon_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2_msgs REQUIRED)
# find_package(class_loader REQUIRED)
find_package(polygon_coverage_msgs REQUIRED)
find_package(polygon_coverage_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
ament_auto_find_build_dependencies()

######
# QT #
######
set(CMAKE_AUTOMOC ON)
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#   ## pull in all required include dirs, define QT_LIBRARIES, etc.
#   include(${QT_USE_FILE})
# else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
# endif()

set (THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp   
  rviz_common 
  pluginlib
  Qt5
)

#############
# LIBRARIES #
#############
# Header files as explained in https://answers.ros.org/question/215487/could-not-load-panel-in-rviz-pluginlibfactory-the-plugin-for-class/
set(HEADER_FILES
  include/rviz_polygon_tool/polygon_tool.h
)
set(SRC_FILES
  src/polygon_tool.cc
)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} 
${QT_LIBRARIES}
# ${class_loader_LIBRARIES}
${pluginlib_LIBRARIES}
${polygon_coverage_msgs_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${polygon_coverage_msgs_INCLUDE_DIRS}
  ${polygon_coverage_geometry_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_redering_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${builtin_interfaces_INCLUDE_DIRS}
  # ${sensor_msgs_INCLUDE_DIRS}
  # ${class_loader_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  /opt/ros/humble/include/class_loader/
  /opt/ros/humble/include/pluginlib/
  /usr/include/OGRE/
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

##########
# EXPORT #
##########
#安装库文件
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

#导出插件描述文件
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
# 在ament中导出项目的头文件、库、目标和依赖项
ament_export_include_directories(include) # 将include目录中的头文件导出
ament_export_libraries(${PROJECT_NAME}) # 将项目名称指定的库导出
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS}) # 将此包的依赖项导出

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
