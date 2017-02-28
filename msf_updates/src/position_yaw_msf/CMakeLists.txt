add_executable(position_yaw_sensor main.cpp)

target_link_libraries(position_yaw_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(position_yaw_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
