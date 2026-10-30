cmake_minimum_required(VERSION 3.0.2)
project(inverse_dynamics_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
  geometry_msgs
  )

# catkin_python_setup()
# rtmbuild2 enumerates ROS services from IDL during CMake configuration.
# Reconfigure on IDL changes so added methods get message-generation targets
# before the updated ROS bridge is compiled by an incremental build.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/idl/InverseDynamicsControllerService.idl")

# initialize rtmbuild (generate_messages is called)
rtmbuild2_init(rtmbuild2)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES InverseDynamicsControllerServiceSkel InverseDynamicsControllerServiceStub
  CATKIN_DEPENDS rtmbuild2
#  DEPENDS system_lib
)

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

###########
## Build ##
###########

add_subdirectory(rtc)

if(CATKIN_ENABLE_TESTING)
  find_package(Threads REQUIRED)
  add_executable(test_direct_torque_gate test/test_direct_torque_gate.cpp)
  target_include_directories(test_direct_torque_gate PRIVATE rtc/InverseDynamicsController)
  target_link_libraries(test_direct_torque_gate ${CMAKE_THREAD_LIBS_INIT})
  add_test(NAME direct_torque_gate COMMAND test_direct_torque_gate)
  add_executable(test_transparency_inputs test/test_transparency_inputs.cpp)
  target_include_directories(test_transparency_inputs PRIVATE rtc/InverseDynamicsController)
  add_test(NAME transparency_inputs COMMAND test_transparency_inputs)
  add_executable(test_wrench_sample_validator test/test_wrench_sample_validator.cpp)
  target_include_directories(test_wrench_sample_validator PRIVATE rtc/ElbowWrenchBridge)
  add_test(NAME wrench_sample_validator COMMAND test_wrench_sample_validator)
endif()
