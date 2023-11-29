cmake_minimum_required(VERSION 3.8)
project(polygon_coverage_solvers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)
ament_auto_find_build_dependencies()
#Used with ament_cmake_auto to automatically generate dependencies

# Add mono to invoke gk_ma.
find_package(PkgConfig REQUIRED)
pkg_check_modules(MONO mono-2 REQUIRED)
#include_directories(${MONO_INCLUDE_DIRS})

# Download gk_ma
set(GKMA_INCLUDE_DIR ${CMAKE_BINARY_DIR}/gk_ma-prefix/src/gk_ma)
#message(${CMAKE_BINARY_DIR})
#message(${PROJECT_SOURCE_DIR})
#message(${CMAKE_CURRENT_SOURCE_DIR})
#message(${CMAKE_INSTALL_PREFIX})
include(ExternalProject)
ExternalProject_Add(
  gk_ma
  URL http://www.cs.nott.ac.uk/~pszdk/gtsp_ma_source_codes.zip
  URL https://polybox.ethz.ch/index.php/s/H4NXeaNPWo6VBrf/download
  DOWNLOAD_NAME gtsp_ma_source_codes.zip
  URL_MD5 765fad8e3746fa3dd9b81be0afb34d35
  #delete patch command
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND
    COMMAND cp ${PROJECT_SOURCE_DIR}/patches/MakefileCpp ./MakefileCpp
    COMMAND cp ${PROJECT_SOURCE_DIR}/patches/MakefileCs ./MakefileCs
    BUILD_COMMAND
    COMMAND $(MAKE) -f MakefileCs BUILD_PATH="${CMAKE_INSTALL_PREFIX}"
    COMMAND $(MAKE) -f MakefileCpp BUILD_PATH="${CMAKE_INSTALL_PREFIX}"
    INSTALL_COMMAND
    COMMAND ${CMAKE_COMMAND} -E create_symlink /usr/lib/libmono-native.so ${CMAKE_INSTALL_PREFIX}/System.Native
)

# Download GTSP test instances.
include(ExternalProject)
ExternalProject_Add(
  gtsp_instances
  URL http://www.cs.nott.ac.uk/~dxk/gtsplib/InstancesBinary.zip
  URL https://polybox.ethz.ch/index.php/s/51iqurpOOQ5cVaJ/download
  DOWNLOAD_NAME InstancesBinary.zip
  URL_MD5 255831bd47de71df8419a54741f0a6be
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} SHARED
  src/gk_ma.cc
  src/combinatorics.cc
  src/boolean_lattice.cc
)

target_link_libraries(${PROJECT_NAME}
  ${MONO_LIBRARIES}  # 你的库依赖项，根据实际情况替换
  ${ament_index_cpp_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rmw_LIBRARIES}
  ${rosidl_typesupport_interface_LIBRARIES}
  ${rcl_interfaces_LIBRARIES}
)

# 添加rclcpp的头文件路径到你的库
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rosidl_typesupport_interface_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${ament_index_cpp_INCLUDE_DIRS}  
    ${MONO_INCLUDE_DIRS}
)

#########
# TESTS #
#########
ament_add_gtest(test_combinatorics test/combinatorics-test.cpp)
target_link_libraries(test_combinatorics ${PROJECT_NAME}) 
ament_add_gtest(test_gk_ma test/gk_ma-test.cpp)
target_link_libraries(test_gk_ma ${PROJECT_NAME} )

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
#ament_export_targets(${PROJECT_NAME})
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
