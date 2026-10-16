cmake_minimum_required(VERSION 3.8)
project(mecamind_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 登记要生成代码的消息文件；DEPENDENCIES 列出消息里用到的外部消息包
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()
