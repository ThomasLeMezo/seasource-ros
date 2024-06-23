cmake_minimum_required(VERSION 3.8)
project(seasource_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fmt REQUIRED)
find_package(SDL2 CONFIG REQUIRED)

## Interfaces
 rosidl_generate_interfaces(${PROJECT_NAME}
         "msg/LogAudioSource.msg"
         "srv/FileSelection.srv"
         "srv/ParametersUpdate.srv"
         DEPENDENCIES std_msgs
         )

 rosidl_get_typesupport_target(cpp_typesupport_target
         ${PROJECT_NAME} "rosidl_typesupport_cpp"
         )

### Executable
add_executable(seasource_audio_node
        src/seasource_audio_node.cpp
)

ament_target_dependencies(seasource_audio_node
        rclcpp
        rosidl_default_generators
        std_msgs
        std_srvs
        fmt
)

target_link_libraries(seasource_audio_node
        "${cpp_typesupport_target}"
        fmt::fmt
        SDL2::SDL2
        SDL2_mixer
)

### Includes
target_include_directories(seasource_audio_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${SDL2_INCLUDE_DIRS}
        ${SDL2_MIXER_INCLUDE_DIRS}
)

### Install
install(TARGETS seasource_audio_node
        DESTINATION lib/${PROJECT_NAME}
)

# Define a custom target to clear the directory
set(INSTALL_DATA_DIR "data")
add_custom_target(clear_install_data_directory
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/data
        COMMENT "Clearing install directory"
)
add_dependencies(seasource_audio_node clear_install_data_directory)

install(
        DIRECTORY data
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
