cmake_minimum_required(VERSION 2.8.3)
project(myworkcell_core)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    fake_ar_publisher
)

catkin_package(
    CATKIN_DEPENDS 
      roscpp 
      fake_ar_publisher
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(vision_node src/vision_node.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})
add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
