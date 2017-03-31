cmake_minimum_required(VERSION 2.8.3)
project(control)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

#set(CMAKE_BUILD_TYPE "Release")
#if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
#else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_BUILD_TYPE "Release")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
#endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()


find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  std_srvs
  smarteye_common
  world_model
  eigen_conversions
)



generate_dynamic_reconfigure_options(cfg/NuFliecontrol.cfg)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation
                 dynamic_reconfigure
	         roscpp 
	         rospy  
		 std_msgs 
		 std_srvs 
		 DEPENDS
)

set(control_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
)

include_directories(${control_includes})

add_library(control  STATIC

)

add_dependencies(control ${catkin_EXPORTED_TARGETS})
target_link_libraries(control
    ${catkin_LIBRARIES}
)

add_executable(control_node src/control.cpp
                            src/main.cpp


)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node
   ${catkin_LIBRARIES}
 )


