
project(comm)
cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Release") #using CMAKE_BUILD_TYPE to set the cmake method [Debug | Release]
# CMAKE_CXX_FLAGS CMake C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fPIE")  #-fopenmp add-on multi-thread openmp support
set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}  "${PROJECT_SOURCE_DIR}/../../3rd_src/gcc_64")


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

#find QT libraries  
find_package(mavlink REQUIRED) 
find_package(Qt5SerialPort REQUIRED) 
find_package(Qt5Widgets REQUIRED)  
find_package(Qt5Core REQUIRED)  
find_package(Qt5Gui REQUIRED)  
find_package(Qt5Positioning REQUIRED) 

file(GLOB source_files "${PROJECT_SOURCE_DIR}/src/*.cpp" )
file(GLOB Headers "include/*.h")
QT5_WRAP_CPP(HEADERS_MOC ${Headers})

find_package(Threads)
set(comm_includes
    include
    include/mavlink
    include/GpsDriver
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS}  ${Qt5Positioning_INCLUDE_DIRS}

    )

include_directories(${comm_includes})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})  
add_library(comm  ${source_files}
    ${HEADERS_MOC}

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

message("CMakePath=${CMAKE_PREFIX_PATH}")
link_directories(${LIBRARY_OUTPUT_PATH}) 
add_executable(comm_node
    ${PROJECT_SOURCE_DIR}/include/GpsDriver/src/gps_helper.cpp
    ${PROJECT_SOURCE_DIR}/include/GpsDriver/GPSProvider.cpp
    ${PROJECT_SOURCE_DIR}/include/GpsDriver/src/ubx.cpp
    ${source_files}
    ${HEADERS_MOC}
    )




target_link_libraries(comm_node
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Qt5Core_LIBRARIES}  ${Qt5Widgets_LIBRARIES}  ${Qt5SerialPort_LIBRARIES} ${Qt5Positioning_LIBRARIES}

    )
add_dependencies(comm_node ${catkin_EXPORTED_TARGETS})
