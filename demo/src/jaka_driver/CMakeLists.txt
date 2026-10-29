cmake_minimum_required(VERSION 3.8)
project(jaka_driver)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Optionally find additional dependencies
# find_package(Boost REQUIRED)

# Declare ROS 2 message and service dependencies
find_package(rosidl_default_generators REQUIRED)

# Declare executable targets
add_executable(jaka_driver src/jaka_driver.cpp)
add_executable(rc_ctrl_node
  src/rc_ctrl_node.cpp
  src/pgi140_gripper.cpp
  src/pgi140_protocol.cpp
)

# Link libraries
ament_target_dependencies(jaka_driver
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  action_msgs
  control_msgs
)


ament_target_dependencies(rc_ctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
)

# For including directories in ROS 2
target_include_directories(jaka_driver
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)



target_include_directories(rc_ctrl_node
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)



# Specify where to find external libraries
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Add the directory to the search path
link_directories(${LIBRARY_DIR})

# Find the library and link it properly
target_link_libraries(jaka_driver ${LIBRARY_DIR}/libjakaAPI.so)
target_link_libraries(rc_ctrl_node ${LIBRARY_DIR}/libjakaAPI.so)
find_package(Threads REQUIRED)
target_link_libraries(rc_ctrl_node Threads::Threads)
set_target_properties(jaka_driver rc_ctrl_node PROPERTIES
  BUILD_RPATH "${LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

# Install executables
install(TARGETS
  jaka_driver
  rc_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

# Keep the ARM64 JAKA SDK beside the nodes that load it. The installed nodes
# use $ORIGIN as their RPATH so the library is found after colcon install.
install(FILES lib/libjakaAPI.so
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(pgi140_protocol_test
    test/pgi140_protocol_test.cpp
    src/pgi140_protocol.cpp
  )
  target_include_directories(pgi140_protocol_test PRIVATE include)
  add_test(NAME pgi140_protocol_test COMMAND pgi140_protocol_test)

  add_executable(orientation_math_test test/orientation_math_test.cpp)
  target_include_directories(orientation_math_test PRIVATE include)
  add_test(NAME orientation_math_test COMMAND orientation_math_test)

  add_executable(teleop_control_test test/teleop_control_test.cpp)
  target_include_directories(teleop_control_test PRIVATE include)
  add_test(NAME teleop_control_test COMMAND teleop_control_test)

  add_executable(teleop_input_test test/teleop_input_test.cpp)
  target_include_directories(teleop_input_test PRIVATE include)
  add_test(NAME teleop_input_test COMMAND teleop_input_test)

  add_executable(pgi140_gripper_startup_test test/pgi140_gripper_startup_test.cpp
    src/pgi140_gripper.cpp src/pgi140_protocol.cpp)
  target_include_directories(pgi140_gripper_startup_test PRIVATE include)
  add_test(NAME pgi140_gripper_startup_test COMMAND pgi140_gripper_startup_test)
endif()

ament_package()
