# mrosuam/src/gui/CMakeLists.txt

add_executable(${PROJECT_NAME}_gui Gui.cpp)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

target_include_directories(${PROJECT_NAME}_gui PUBLIC ${PROJECT_SOURCE_DIR}/include)

ament_target_dependencies(${PROJECT_NAME}_gui rclcpp std_msgs geometry_msgs mavros_msgs camera_ros)

target_link_libraries(${PROJECT_NAME}_gui
  imgui
  implot
  ${CMAKE_DL_LIBS}
  jpgd
  glfw
  OpenGL::GL
)


install(TARGETS ${PROJECT_NAME}_gui DESTINATION lib/${PROJECT_NAME})
