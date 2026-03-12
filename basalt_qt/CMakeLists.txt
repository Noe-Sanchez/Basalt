cmake_minimum_required(VERSION 3.8)
project(basalt_qt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(formation_creator src/creator.cpp)
ament_target_dependencies(formation_creator rclcpp nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(formation_creator Qt6::Core Qt6::Widgets)

install(TARGETS
  formation_creator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
