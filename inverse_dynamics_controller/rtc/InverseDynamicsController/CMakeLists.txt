set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  cpp_filters
  hrpsys_ros_bridge
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(hrpsys_base hrpsys-base REQUIRED)
find_library(HRPSYS_BASE_STUB_LIBRARY hrpsysBaseStub HINTS ${hrpsys_base_LIBRARY_DIRS})
if(NOT HRPSYS_BASE_STUB_LIBRARY)
  message(FATAL_ERROR "hrpsysBaseStub is required for OpenHRP::TimedLongSeqSeq")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )
link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_library(InverseDynamicsController SHARED
  InverseDynamicsController.cpp
  InverseDynamicsControllerService_impl.cpp
  )
target_link_libraries(InverseDynamicsController
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${HRPSYS_BASE_STUB_LIBRARY}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  # RobotHardwareServiceSkel RobotHardwareServiceStub
  )
set_target_properties(InverseDynamicsController PROPERTIES PREFIX "")
add_dependencies(InverseDynamicsController RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild2_genidl

add_executable(InverseDynamicsControllerComp InverseDynamicsControllerComp.cpp)
target_link_libraries(InverseDynamicsControllerComp InverseDynamicsController)

install(TARGETS InverseDynamicsController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
