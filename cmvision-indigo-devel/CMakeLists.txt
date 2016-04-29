cmake_minimum_required(VERSION 2.8.3)
project(cmvision)

find_package(catkin REQUIRED 
  COMPONENTS 
  cv_bridge
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  Blob.msg
  Blobs.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cmvision
	src/cmvision.cc
	src/conversions.c
	src/cmvision_color_blob_finder.cpp
	src/main.cpp
)
add_dependencies(cmvision
  cmvision_gencpp
)
target_link_libraries(cmvision 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(colorgui
	src/cmvision.cc
	src/conversions.c
	src/color_gui.cpp
)
target_link_libraries(colorgui 
  fltk
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${wxWidgets_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS cmvision colorgui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES cmvision.launch colors.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

