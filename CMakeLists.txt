cmake_minimum_required(VERSION 3.10)
project(omni_robot_ros VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(OMNI_BUILD_ROS "Build the ROS Melodic node (Linux/catkin)" ON)

add_library(omni_protocol src/command_encoder.cpp src/command_gate.cpp)
target_include_directories(omni_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(MSVC)
  target_compile_options(omni_protocol PRIVATE /W4)
else()
  target_compile_options(omni_protocol PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(UNIX)
  find_package(Threads REQUIRED)
  add_library(omni_cansend src/cansend_transport.cpp)
  target_include_directories(omni_cansend PUBLIC include)
  target_link_libraries(omni_cansend PUBLIC omni_protocol Threads::Threads)
endif()

if(OMNI_BUILD_ROS)
  if(NOT UNIX)
    message(FATAL_ERROR "ROS node requires Linux. Use -DOMNI_BUILD_ROS=OFF for host tests.")
  endif()
  find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs std_msgs)
  catkin_package(INCLUDE_DIRS include LIBRARIES omni_protocol
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs)
  add_executable(omni_can_bridge src/omni_can_bridge.cpp)
  target_include_directories(omni_can_bridge PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(omni_can_bridge omni_cansend ${catkin_LIBRARIES})
  install(TARGETS omni_protocol omni_cansend omni_can_bridge
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(core_test test/core_test.cpp)
  target_link_libraries(core_test omni_protocol)
  add_test(NAME core_test COMMAND core_test)
  if(UNIX)
    add_executable(fake_cansend test/fake_cansend.cpp)
    add_executable(transport_test test/transport_test.cpp)
    target_link_libraries(transport_test omni_cansend)
    add_test(NAME transport_test COMMAND transport_test $<TARGET_FILE:fake_cansend>)
  endif()
endif()
