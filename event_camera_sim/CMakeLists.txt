cmake_minimum_required(VERSION 3.8)
project(event_camera_sim)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Event.msg"
  "msg/EventArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_python_install_package(event_camera_sim_py)

install(PROGRAMS
  event_camera_sim_py/image_to_events.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
