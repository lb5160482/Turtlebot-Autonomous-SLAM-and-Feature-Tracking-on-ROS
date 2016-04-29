cmake_minimum_required(VERSION 2.8.3)
project(tracker)

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

add_executable(killTracking src/killTracking.cpp)
target_link_libraries(killTracking ${catkin_LIBRARIES})

add_executable(startTracking src/startTracking.cpp)
target_link_libraries(startTracking ${catkin_LIBRARIES})

