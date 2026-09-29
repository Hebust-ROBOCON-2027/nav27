cmake_minimum_required(VERSION 3.8)
project(rc27_serial)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# 串口驱动 + 底盘实现（协议解析、组帧、IChassis 串口实现）
# ---------------------------------------------------------------------------
add_library(rc27_serial_port SHARED
  src/serial_port.cpp
  src/serial_chassis.cpp
)
target_include_directories(rc27_serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rc27_serial_port rclcpp)

# ---------------------------------------------------------------------------
# ROS 节点：订阅 cmd_vel -> IChassis::setVelocity()
# ---------------------------------------------------------------------------
add_executable(chassis_bridge_node
  src/chassis_bridge_node.cpp
)
target_link_libraries(chassis_bridge_node rc27_serial_port)
ament_target_dependencies(chassis_bridge_node rclcpp geometry_msgs std_msgs)

install(TARGETS
  rc27_serial_port
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  chassis_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
