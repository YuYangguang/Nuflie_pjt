cmake_minimum_required(VERSION 2.8.3)
project(px4controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_extras
  geometry_msgs
  tf2_ros
  dynamic_reconfigure
  message_generation
  world_model
  smarteye_common
  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)



add_message_files(
   FILES
   count.msg
   download.msg
   upload.msg
)

generate_messages( 
    DEPENDENCIES 
    geometry_msgs
    std_msgs
    
)
include(EnableCXX11)


add_library(px4controller  STATIC

)
add_dependencies(px4controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4controller
    ${catkin_LIBRARIES}
)



catkin_package(
  INCLUDE_DIRS   include
  CATKIN_DEPENDS dynamic_reconfigure message_filters message_generation  roscpp rospy  std_msgs std_srvs
  LIBRARIES px4controller
)

set(px4controller_includes
               include
               ${catkin_INCLUDE_DIRS}
)

include_directories(${px4controller_includes}
                    ${catkin_INCLUDE_DIRS}
)



add_executable(px4controller_node
  src/main.cpp
  src/px4Controller.cpp)
target_link_libraries(px4controller_node
   ${catkin_LIBRARIES}
 )



