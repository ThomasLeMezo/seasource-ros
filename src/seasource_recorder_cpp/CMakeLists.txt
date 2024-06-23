cmake_minimum_required(VERSION 3.8)
project(seasource_recorder_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

## Interfaces

### Executable
add_executable(recorder
        src/recorder_node.cpp
)

ament_target_dependencies(recorder
        rclcpp
        rosidl_default_generators
        std_srvs
)

target_link_libraries(recorder
        "${cpp_typesupport_target}"
)

### Includes
target_include_directories(recorder PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS recorder
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
