cmake_minimum_required(VERSION 2.8.3)
project(vr_planning)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
)

find_package(Boost REQUIRED)

message(STATUS "BOOST_INCLUDE_DIR='${BOOST_INCLUDE_DIR}'")

include_directories(
    include
    ${BOOST_INCLUDE_DIR}
)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS
)


include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(client_node
    src/client_node.cpp
)

target_link_libraries(client_node ${catkin_LIBRARIES} ${BOOST_LIBRARIES})