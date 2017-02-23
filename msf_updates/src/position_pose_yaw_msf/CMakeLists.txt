add_executable(position_pose_yaw_sensor main.cpp)

target_link_libraries(position_pose_yaw_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(position_pose_yaw_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
