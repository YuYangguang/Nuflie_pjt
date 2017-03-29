
project(comm)
cmake_minimum_required(VERSION 2.8.3)
#set(CMAKE_BUILD_TYPE "Release") #using CMAKE_BUILD_TYPE to set the cmake method [Debug | Release]
# CMAKE_CXX_FLAGS CMake C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fPIE")  #-fopenmp add-on multi-thread openmp support
set(CMAKE_AUTOMOC ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
aux_source_directory(. SRC_LIST)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  smarteye_common
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  mavros
)

find_package(Qt5SerialPort REQUIRED) 
#find QT libraries  
find_package(Qt5Widgets REQUIRED)  
find_package(Qt5Core REQUIRED)  
find_package(Qt5Gui REQUIRED)  

FILE(GLOB MOCS "include/*.h")
QT5_WRAP_CPP(HEADERS_MOC ${MOCS})


INCLUDE_DIRECTORIES (${CMAKE_CURRENT_BINARY_DIR})



find_package(Threads)

set(comm_includes
               include
               ${catkin_INCLUDE_DIRS}
               ${Boost_INCLUDE_DIRS}
${Qt5Core_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS}
${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS}


)

include_directories(${comm_includes})

add_library(comm  STATIC    
                  src/main.cpp                   
                  #src/MavlinkHandler.cpp

)
add_dependencies(comm ${catkin_EXPORTED_TARGETS})
target_link_libraries(comm
    ${catkin_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS   include
  CATKIN_DEPENDS dynamic_reconfigure  message_generation  roscpp rospy  std_msgs std_srvs mavros
  LIBRARIES comm
)



add_executable(comm_node
              src/main.cpp
              src/MavlinkHandler.cpp
              ${HEADERS_MOC})

              



target_link_libraries(comm_node
    ${catkin_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
${Qt5Core_LIBRARIES}
    comm
)
add_dependencies(comm_node ${catkin_EXPORTED_TARGETS})


