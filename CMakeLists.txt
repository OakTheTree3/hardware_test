cmake_minimum_required(VERSION 3.8)
project(hardware_test LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
  std_srvs
  example_interfaces
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(rft_sensor_serial 
  hardware_src/rft_sensor_serial.cpp
  hardware_src/RFT_UART_SAMPLE.cpp
  hardware_src/RFT_IF_PACKET_Rev1.2.cpp
)

add_executable(cpp_pid 
    hardware_src/cpp_pid.cpp
)

add_executable(cpp_motor
    hardware_src/cpp_motor.cpp
    include/arduino_comms.hpp
)

target_include_directories(rft_sensor_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rft_sensor_serial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(rft_sensor_serial 
  rclcpp 
  geometry_msgs
  std_srvs
  example_interfaces)

ament_target_dependencies(cpp_pid
    rclcpp
    geometry_msgs)

ament_target_dependencies(cpp_motor
    rclcpp
    geometry_msgs
    std_msgs)

target_include_directories(cpp_motor PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cpp_motor PUBLIC cxx_std_17)
target_link_libraries(cpp_motor ${catkin_LIBRARIES} serial)

install(
    TARGETS rft_sensor_serial cpp_pid cpp_motor
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY config include launch
    DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()