cmake_minimum_required(VERSION 3.0.2)
project(handle_interface)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS #system
        roslib
        roscpp
        message_generation
        std_msgs
        wam_srvs
)

add_message_files(DIRECTORY msg
  FILES
  handle_state.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
add_executable(handle_controller src/handle_controller.cpp)
add_dependencies(handle_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(handle_controller ${catkin_LIBRARIES})

add_executable(handle_sim src/handle_sim.cpp)
add_dependencies(handle_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(handle_sim ${catkin_LIBRARIES})
