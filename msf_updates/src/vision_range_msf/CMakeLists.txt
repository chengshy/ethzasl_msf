add_executable(vision_range_sensor main.cpp)

target_link_libraries(vision_range_sensor ${catkin_LIBRARIES})

add_dependencies(vision_range_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
