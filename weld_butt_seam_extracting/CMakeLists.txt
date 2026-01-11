cmake_minimum_required(VERSION 3.10)
project(weld_butt_seam_extracting)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io features filters segmentation sample_consensus kdtree)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# 编译所有源文件
set(SOURCES
  src/weld_butt.cpp
  src/SeamSepAllPlane.cpp
  src/SeamCluAllPlane.cpp
  src/SeamDownSmape.cpp
  src/SeamTwoPlane.cpp
  src/SeamDownSmapeT.cpp
  src/SeamFourPlane.cpp
  src/SeamFindSelf.cpp
  src/SeamRevolve.cpp
  src/SeamCoefficient.cpp
  src/SeamIntersectLine.cpp
  src/SeamDrawLine.cpp
  src/SeamAllIntersectPoint.cpp
  src/SeamEndShow.cpp
  src/SeamLinkTwo.cpp
)

# 创建可执行文件
add_executable(weld_butt_seam_extracting_node ${SOURCES})

# 设置依赖
set(DEPENDENCIES
  rclcpp
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(weld_butt_seam_extracting_node
  ${DEPENDENCIES}
)

# 链接PCL和OpenCV库
target_link_libraries(weld_butt_seam_extracting_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# 安装可执行文件
install(TARGETS
  weld_butt_seam_extracting_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
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
