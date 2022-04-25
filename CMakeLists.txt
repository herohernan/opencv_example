cmake_minimum_required(VERSION 2.8.3)
project(opencv_example)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge # new
  image_transport # new
  roscpp
  sensor_msgs # new
  std_msgs
)

find_package(OpenCV REQUIRED) # OPENCV

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_example
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS} # OPENCV
  ${catkin_INCLUDE_DIRS}
)

# NODE 1. opencv_template_node
add_executable(opencv_template_node src/opencv_template_node.cpp)
 target_link_libraries(opencv_template_node
   ${OpenCV_LIBRARIES} # OPENCV
   ${catkin_LIBRARIES} # dejarlo
 )

# NODE 2. opencv_template_node_hh
add_executable(opencv_template_node_hh src/opencv_template_node_hh.cpp)
 target_link_libraries(opencv_template_node_hh
   ${OpenCV_LIBRARIES} # OPENCV
   ${catkin_LIBRARIES} # dejarlo
 )

# NODE 3. opencv_grayImage_hh
add_executable(opencv_grayImage_hh src/opencv_grayImage_hh.cpp)
 target_link_libraries(opencv_grayImage_hh
   ${OpenCV_LIBRARIES} # OPENCV
   ${catkin_LIBRARIES} # dejarlo
 )

# NODE 4. opencv_change_contrast_hh
add_executable(opencv_change_contrast_hh src/opencv_change_contrast_hh.cpp)
 target_link_libraries(opencv_change_contrast_hh
   ${OpenCV_LIBRARIES} # OPENCV
   ${catkin_LIBRARIES} # dejarlo
 )

# NODE 5. opencv_smoothingImages_hh
add_executable(opencv_smoothingImages_hh src/opencv_smoothingImages_hh.cpp)
 target_link_libraries(opencv_smoothingImages_hh
   ${OpenCV_LIBRARIES} # OPENCV
   ${catkin_LIBRARIES} # dejarlo
 )

# NODE 6. opencv_houghCircleTransform_hh
add_executable(opencv_houghCircleTransform_hh src/opencv_houghCircleTransform_hh.cpp)
 target_link_libraries(opencv_houghCircleTransform_hh
   ${OpenCV_LIBRARIES} # OPENCV
   ${catkin_LIBRARIES} # dejarlo
 )


