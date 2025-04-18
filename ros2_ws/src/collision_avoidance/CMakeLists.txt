cmake_minimum_required(VERSION 3.8)
project(collision_avoidance_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -g -ggdb -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${PCL_COMMON_INCLUDE_DIRS})

add_executable(collision_avoidance_base src/collision_avoidance.cpp)
target_include_directories(collision_avoidance_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(collision_avoidance_base ${PCL_LIBRARIES})
target_compile_features(collision_avoidance_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  collision_avoidance_base
  "rclcpp"
  "pcl_conversions"
  "sensor_msgs"
  "std_msgs"
  "geometry_msgs"
)

install(TARGETS collision_avoidance_base
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
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
ament_export_include_directories(include)
ament_package()
