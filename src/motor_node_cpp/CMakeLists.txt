cmake_minimum_required(VERSION 3.8)
project(motor_node_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xycar_msgs REQUIRED)

# 프로젝트 루트 디렉토리 경로 변수 설정
set(SEAME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../2025-SEAME)

# include 및 link 경로 안전하게 설정
include_directories(${SEAME_DIR}/include)
link_directories(${SEAME_DIR})

add_executable(motor_node src/motor_node.cpp)

ament_target_dependencies(motor_node rclcpp xycar_msgs)

install(TARGETS
  motor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()