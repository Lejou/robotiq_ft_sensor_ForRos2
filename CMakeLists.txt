cmake_minimum_required(VERSION 3.5)
project(robotiq_ft_sensor)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
# find_package(catkin REQUIRED

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
# Instead of REQUIRED_COMPONENTS std_msgs
#find_package(rosidl_default_generators REQUIRED)
# Instead of REQUIRED_COMPONENTS message_generation

set(msg_files
   "msg/Ftsensor.msg"
)
set(srv_files
   "srv/Sensoraccessor.srv"
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs std_srvs builtin_interfaces 
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(rq_sensor nodes/rq_sensor.cpp src/rq_sensor_com.cpp src/rq_sensor_state.cpp)
target_include_directories(rq_sensor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

rosidl_target_interfaces(rq_sensor
${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(rq_sensor
  rclcpp
  geometry_msgs
  # std_msgs
  # std_srvs
  # rosidl_default_generators
  )
  
install(TARGETS 
  rq_sensor
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY urdf/
  DESTINATION urdf
)

install(DIRECTORY meshes/
  DESTINATION meshes
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
# ament_export_dependencies(std_msgs)
# ament_export_dependencies(std_srvs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
