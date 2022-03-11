cmake_minimum_required(VERSION 2.8.3)
project(vr_planning)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
)

include_directories(
    include
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

# add_executable(path_planning_node
#     src/path_planning_node.py
# )

target_link_libraries(client_node ${catkin_LIBRARIES})
# target_link_libraries(path_planning_node ${catkin_LIBRARIES})