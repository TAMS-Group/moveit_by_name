cmake_minimum_required(VERSION 3.0.2)
project(moveit_by_name)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  message_generation
  roscpp
)

add_message_files(
  FILES
  Command.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/moveit_by_name.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
