add_executable(vision_sensor main.cpp)

target_link_libraries(vision_sensor ${catkin_LIBRARIES})

add_dependencies(vision_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
