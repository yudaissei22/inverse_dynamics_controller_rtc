find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS})
add_definitions(-Wno-deprecated)

add_library(ElbowWrenchBridge SHARED ElbowWrenchBridge.cpp)
set_target_properties(ElbowWrenchBridge PROPERTIES PREFIX "")
target_link_libraries(ElbowWrenchBridge ${catkin_LIBRARIES} ${openrtm_aist_LIBRARIES})
add_executable(ElbowWrenchBridgeComp ElbowWrenchBridgeComp.cpp)
target_link_libraries(ElbowWrenchBridgeComp ElbowWrenchBridge)
install(TARGETS ElbowWrenchBridge ElbowWrenchBridgeComp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
