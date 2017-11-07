cmake_minimum_required(VERSION 2.8)
project(rcprg_ros_utils)

find_package(catkin REQUIRED COMPONENTS
   roscpp
  )


# Export package information (replaces catkin_package() macro) 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rcprg_ros_utils
)

### Orocos Package Exports and Install Targets ###

catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

include_directories(include)

add_library(rcprg_ros_utils
    src/marker_publisher.cpp)

target_link_libraries(rcprg_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### Orocos Package Exports and Install Targets ###

install(TARGETS rcprg_ros_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

catkin_install_python(PROGRAMS
    scripts/pose_int_marker.py
    scripts/publish_camera_frustum.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

