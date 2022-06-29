cmake_minimum_required(VERSION 3.8)
project(nirmitsu_ros2_flowgraph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(NodeEditor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED
  COMPONENTS
  Core
  Widgets
  Gui
  OpenGL
)
find_package(std_msgs REQUIRED)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

#===============================================================================
file(GLOB_RECURSE SRC_FILES  src/*.cpp )
add_executable(nirmitsu ${SRC_FILES})

target_link_libraries(nirmitsu
  PRIVATE
    rclcpp::rclcpp
    ${NodeEditor_LIBRARIES}
    ${QT5_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

target_include_directories(nirmitsu
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${NodeEditor_INCLUDE_DIRS}
    ${QT5_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

target_compile_features(nirmitsu INTERFACE cxx_std_17)

#===============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#===============================================================================
install(
  TARGETS nirmitsu
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
