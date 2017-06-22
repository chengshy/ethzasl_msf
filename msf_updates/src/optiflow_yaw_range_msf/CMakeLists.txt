add_executable(optiflow_yaw_range_sensor main.cpp)

target_link_libraries(optiflow_yaw_range_sensor ${catkin_LIBRARIES})

add_dependencies(optiflow_yaw_range_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
