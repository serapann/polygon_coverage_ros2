cmake_minimum_required(VERSION 3.8)
project(test1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/test1.cc
)
target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
PUBLIC
  include
  ${rclcpp_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}

)

add_executable(test1_node
  src/test1_node.cc
)
target_link_libraries(test1_node ${PROJECT_NAME})

add_executable(test2_node
  src/test2_node.cc
)
target_link_libraries(test2_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        # LIBRARY DESTINATION lib/${PROJECT_NAME}????
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS 
  test1_node test2_node
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
  parameters/ 
  DESTINATION
    share/${PROJECT_NAME}/parameters
)
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION include/${PROJECT_NAME}
#         FILES_MATCHING PATTERN "*.h"
#         )

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
