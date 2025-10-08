# mrosuam/src/resource_monitor/CMakeLists.txt

add_executable(${PROJECT_NAME}_resource_monitor ResourceMonitor.cpp)

ament_target_dependencies(${PROJECT_NAME}_resource_monitor rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}_resource_monitor DESTINATION lib/${PROJECT_NAME})
