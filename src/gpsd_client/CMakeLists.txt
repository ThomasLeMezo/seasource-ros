cmake_minimum_required(VERSION 3.8)
project(gpsd_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/GpsFix.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Excutable
add_executable(gpsd_node src/gpsd_node.cpp)
ament_target_dependencies(gpsd_node rclcpp std_msgs)
target_link_libraries(gpsd_node "${cpp_typesupport_target}" gps)

### Includes

target_include_directories(gpsd_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS gpsd_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()