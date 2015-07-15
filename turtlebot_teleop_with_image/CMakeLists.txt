cmake_minimum_required(VERSION 2.8)
project(turtlebot_teleop_with_image)

find_package(catkin REQUIRED)
catkin_package()

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()

find_package(catkin REQUIRED camera_calibration_parsers cv_bridge image_transport message_filters nodelet rosconsole roscpp)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

# Deal with the GUI's
if(ANDROID)
  return()
endif()

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(SYSTEM ${GTK2_INCLUDE_DIRS})

# Nodelet library
add_library(interaction_view src/nodelets/image_nodelet.cpp src/nodelets/window_thread.cpp)
target_link_libraries(interaction_view ${catkin_LIBRARIES}
                                 ${GTK_LIBRARIES}
                                 ${GTK2_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 ${Boost_LIBRARIES}
)
install(TARGETS interaction_view
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Image viewers
add_executable(interaction_view_exe src/nodes/interaction_view.cpp)
SET_TARGET_PROPERTIES(interaction_view_exe PROPERTIES OUTPUT_NAME interaction_view)
target_link_libraries(interaction_view_exe ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

install(TARGETS interaction_view_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
