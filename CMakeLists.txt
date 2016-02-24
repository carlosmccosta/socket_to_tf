cmake_minimum_required(VERSION 2.8.3)
project(socket_to_tf)



#######################################################################################################################
## packages
#######################################################################################################################

# remove logging (ROSCONSOLE_SEVERITY_NONE=5)
# set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=5 ${ROS_COMPILE_FLAGS}")

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    geometry_msgs
    roscpp
    rosconsole
    rostime
    tf2
    tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Boost REQUIRED)

find_path ( ZMQ_INCLUDE_DIR zmq.h )
find_library ( ZMQ_LIBRARY NAMES zmq )
set ( ZMQ_LIBRARIES ${ZMQ_LIBRARY} )
set ( ZMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR} )
include ( FindPackageHandleStandardArgs )
find_package_handle_standard_args ( ZMQ DEFAULT_MSG ZMQ_LIBRARY ZMQ_INCLUDE_DIR )


#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        socket_to_tf
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

add_library(socket_to_tf
    src/socket_to_tf.cpp
)



#==================
# executables
#==================

add_executable(socket_to_tf_node
    src/socket_to_tf_node.cpp
)


add_executable(socket_transform_publisher
    src/socket_transform_publisher.cpp
)


#==================
# library link
#==================

target_link_libraries(socket_to_tf
    ${ZMQ_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(socket_to_tf_node
    socket_to_tf
    ${catkin_LIBRARIES}
)

target_link_libraries(socket_transform_publisher
    ${ZMQ_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
