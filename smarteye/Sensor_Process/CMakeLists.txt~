cmake_minimum_required(VERSION 2.8.3)
project(sensor_process)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

#set(CMAKE_BUILD_TYPE "Release")
#if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
#else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_BUILD_TYPE "Release")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
#endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  dynamic_reconfigure
  message_filters
  world_model
  nuflie_common
)


add_message_files(FILES

 )

generate_messages(DEPENDENCIES std_msgs)



set(NuFlie_sensor_includes
               include
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include/sensor_process
)

include_directories(${NuFlie_sensor_includes})

add_library(sensor_process  STATIC

)

add_dependencies(sensor_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_process
    ${catkin_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation
	         roscpp 
	         rospy  
		 std_msgs 
		 std_srvs 
                 dynamic_reconfigure 
                 message_filters
		 DEPENDS
  LIBRARIES      sensor_process
)

add_executable(sensor_process_node src/main.cpp
                                  src/sensor_process.cpp
                                  src/camera.cpp
                                  src/GPS.cpp
                                  src/IMU.cpp
                                  src/sonar.cpp

)
add_dependencies(sensor_process_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_process_node
   ${catkin_LIBRARIES}
   sensor_process
 )



