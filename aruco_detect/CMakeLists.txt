
cmake_minimum_required(VERSION 2.8.3)
project(aruco_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  fiducial_msgs
  dynamic_reconfigure
  nodelet
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

catkin_package(INCLUDE_DIRS DEPENDS OpenCV)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/aruco.cpp
  src/aruco_detect.cpp
  src/aruco_detect_nodelet.cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(aruco_detect_node src/aruco_detect_node.cpp)
set_target_properties(aruco_detect_node PROPERTIES OUTPUT_NAME aruco_detect)
target_link_libraries(aruco_detect_node ${PROJECT_NAME})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS aruco_detect aruco_detect_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/create_markers.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
) 

###########
## Tests ##
###########

if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)

        # Tests need c++11
        add_definitions(-std=c++11)
        
        add_rostest_gtest(aruco_images_test 
          test/aruco_images.test 
          test/aruco_images_test.cpp)
        target_link_libraries(aruco_images_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
