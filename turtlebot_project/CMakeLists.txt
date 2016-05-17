cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ar_track_alvar
)


catkin_package(
  CATKIN_DEPENDS roscpp 
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(ARtracker src/ARtracker.cpp)
target_link_libraries(ARtracker ${catkin_LIBRARIES})

add_executable(colorTracker src/colorTracker.cpp)
target_link_libraries(colorTracker ${catkin_LIBRARIES})

add_executable(mapsaver src/mapsaver.cpp)
target_link_libraries(mapsaver ${catkin_LIBRARIES})

add_executable(startArTracking src/startArTracking.cpp)
target_link_libraries(startArTracking ${catkin_LIBRARIES})

add_executable(startColorTracking src/startColorTracking.cpp)
target_link_libraries(startColorTracking ${catkin_LIBRARIES})

add_executable(viewNavigation src/viewNavigation.cpp)
target_link_libraries(viewNavigation ${catkin_LIBRARIES})

add_executable(startVoice src/startVoice.cpp)
target_link_libraries(startVoice ${catkin_LIBRARIES})


