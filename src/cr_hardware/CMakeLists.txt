cmake_minimum_required(VERSION 3.16)
project(cr_hardware LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wno-unused-function)
endif()

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

set(THIS_PACKAGE_INCLUDE_DEPENDS
  # start ros2 control required
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # end ros2 control required 
  rclpy
  sensor_msgs
  geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  include/cr_hardware
)

add_library(
  roboclaw
  SHARED
  src/roboclaw.c
)
set_target_properties(roboclaw PROPERTIES LINKER_LANGUAGE C)

# compile custom ros2 control SystemInterface plugin
add_library(
  cr_hardware
  SHARED
  src/differential_drive.cpp
)
target_compile_features(cr_hardware PUBLIC cxx_std_17)
target_include_directories(cr_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cr_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(cr_hardware PUBLIC roboclaw)

add_executable(test_roboclaw 
  src/test_roboclaw.c
)
target_include_directories(test_roboclaw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_roboclaw PUBLIC cr_hardware roboclaw)

## Declare a C++ library
add_library(bno055 src/bno055.cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(bno055_node src/bno055_node.cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(bno055_node
#    bno055
#    ${catkin_LIBRARIES}
# )

set_property(TARGET bno055 PROPERTY LINKER_LANGUAGE CXX)
add_executable(bno055_calibrate src/bno055_calibrate.cpp)
target_include_directories(bno055_calibrate PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bno055_calibrate bno055)

add_executable(bno055_driver_node src/bno055_node.cpp)
target_include_directories(bno055_driver_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bno055_driver_node rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(bno055_driver_node bno055)

pluginlib_export_plugin_description_file(hardware_interface cr_hardware.xml)

# libraries and executables
install(TARGETS
  cr_hardware
  roboclaw
  test_roboclaw
  bno055_calibrate
  bno055_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
# includes
install(
  DIRECTORY include/
  DESTINATION include/cr_hardware
)
# launch and urdf files
install(
  DIRECTORY description/ros2_control description/urdf config launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS cr_hardware roboclaw
  EXPORT export_cr_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# python scripts
install(
  PROGRAMS 
    ${PROJECT_NAME}/joy_to_twist_node.py
    ${PROJECT_NAME}/camera.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_cr_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
