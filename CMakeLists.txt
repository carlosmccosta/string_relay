cmake_minimum_required(VERSION 2.8.3)
project(string_relay)



#######################################################################################################################
## packages
#######################################################################################################################

# remove logging (ROSCONSOLE_SEVERITY_NONE=5)
# set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=5 ${ROS_COMPILE_FLAGS}")

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    std_msgs
    roscpp
    rosconsole
    rostime
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Boost REQUIRED iostreams serialization)

find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)
set(ZMQ_LIBRARIES ${ZMQ_LIBRARY})
set( ZMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR})
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZMQ DEFAULT_MSG ZMQ_LIBRARY ZMQ_INCLUDE_DIR)


#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        string_relay
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
        ZMQ
)



#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================

include_directories(
    include
    ${ZMQ_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(string_relay
    src/string_relay.cpp
)



#==================
# executables
#==================

add_executable(string_relay_node
    src/string_relay_node.cpp
)


#==================
# library link
#==================

target_link_libraries(string_relay
    ${ZMQ_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(string_relay_node
    string_relay
    ${catkin_LIBRARIES}
)
