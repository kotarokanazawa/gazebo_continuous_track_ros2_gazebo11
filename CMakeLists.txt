cmake_minimum_required(VERSION 3.8)
project(gazebo_continuous_track_ros2_gazebo11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo REQUIRED)

set(gazebo_ode_INCLUDE_DIRS "")
foreach(gazebo_INCLUDE_DIR ${GAZEBO_INCLUDE_DIRS})
  list(APPEND gazebo_ode_INCLUDE_DIRS ${gazebo_INCLUDE_DIR}/gazebo)
endforeach()

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ode_INCLUDE_DIRS}
)

add_library(ContinuousTrack SHARED
  src/gazebo_continuous_track.cpp
)
ament_target_dependencies(ContinuousTrack
  ament_index_cpp
)
target_link_libraries(ContinuousTrack
  ${GAZEBO_LIBRARIES}
)

add_library(ContinuousTrackSimple SHARED
  src/gazebo_continuous_track_simple.cpp
)
ament_target_dependencies(ContinuousTrackSimple
  ament_index_cpp
)
target_link_libraries(ContinuousTrackSimple
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  ContinuousTrack
  ContinuousTrackSimple
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  sdf
  urdf_xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  env-hooks/gazebo_continuous_track_ros2_gazebo11.dsv.in
  env-hooks/gazebo_continuous_track_ros2_gazebo11.sh.in
)

ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp gazebo)
ament_package()
