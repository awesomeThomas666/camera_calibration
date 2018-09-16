cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration)
add_compile_options(-std=c++11)
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
# )
#find_package(catkin REQUIRED COMPONENTS  image_transport roscpp cv_bridge sensor_msgs nodelet camera_info_manager roslint)
find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread)


# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )

add_executable(camera_calibration src/camera_calibration.cpp)
# add_dependencies(robot_camera_node cv_camera)
# add_dependencies(cv_camera_nodelet cv_camera)
target_link_libraries(camera_calibration
  ${OpenCV_LIBS}
  )
# target_link_libraries(robot_camera_node
#    ${catkin_LIBRARIES}
#    cv_camera
#  )




