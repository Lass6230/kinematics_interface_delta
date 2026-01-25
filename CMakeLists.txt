cmake_minimum_required(VERSION 3.8)
project(kinematics_interface_delta)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest QUIET)
find_package(kinematics_interface QUIET)
find_package(pluginlib QUIET)

# locate kinematics_interface header explicitly
find_path(KINEMATICS_INTERFACE_INCLUDE_DIR
  NAMES kinematics_interface/kinematics_interface/kinematics_interface.hpp
)

include_directories(include)

add_executable(test_delta_kinematics src/test_delta_kinematics.cpp src/delta_kinematics.cpp)
target_link_libraries(test_delta_kinematics Eigen3::Eigen)
ament_target_dependencies(test_delta_kinematics rclcpp)

if(KINEMATICS_INTERFACE_INCLUDE_DIR AND pluginlib_FOUND)
  add_compile_definitions(KINEMATICS_INTERFACE_AVAILABLE)
  # plugin target (simple wrapper). Only built when kinematics_interface headers and pluginlib are available
  add_library(delta_kinematics_plugin SHARED src/delta_plugin.cpp src/delta_kinematics.cpp)
  target_include_directories(delta_kinematics_plugin PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${KINEMATICS_INTERFACE_INCLUDE_DIR})
  target_link_libraries(delta_kinematics_plugin Eigen3::Eigen)
  ament_target_dependencies(delta_kinematics_plugin pluginlib rclcpp)
  install(TARGETS delta_kinematics_plugin 
    EXPORT delta_kinematics_plugin_targets
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  # Register the plugin with pluginlib so it can be discovered
  pluginlib_export_plugin_description_file(kinematics_interface_delta kinematics_interface_delta_plugins.xml)
else()
  message(STATUS "kinematics_interface headers or pluginlib not found; delta_kinematics_plugin will not be built")
endif()

# Install plugin description so pluginlib-based loaders (e.g., controllers) can find this plugin
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/kinematics_interface_delta_plugins.xml"
  DESTINATION share/${PROJECT_NAME}
)

target_include_directories(test_delta_kinematics PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  test_delta_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/kinematics_interface_delta
  DESTINATION include
)

if(ament_cmake_gtest_FOUND)
  ament_add_gtest(test_delta_gtest test/test_delta_gtest.cpp src/delta_kinematics.cpp)
  if(TARGET test_delta_gtest)
    target_include_directories(test_delta_gtest PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_delta_gtest Eigen3::Eigen)
  endif()
endif()

# pluginlib-load integration test: DISABLED - has compilation issues with kinematics_interface headers
# if(KINEMATICS_INTERFACE_INCLUDE_DIR AND pluginlib_FOUND AND ament_cmake_gtest_FOUND)
#   add_definitions(-DKINEMATICS_INTERFACE_AVAILABLE)
#   ament_add_gtest(test_pluginlib_load test/test_pluginlib_load.cpp)
#   if(TARGET test_pluginlib_load)
#     target_include_directories(test_pluginlib_load PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${KINEMATICS_INTERFACE_INCLUDE_DIR})
#     target_link_libraries(test_pluginlib_load Eigen3::Eigen)
#     ament_target_dependencies(test_pluginlib_load pluginlib rclcpp)
#   endif()
# endif()

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp)

if(KINEMATICS_INTERFACE_INCLUDE_DIR AND pluginlib_FOUND)
  install(EXPORT delta_kinematics_plugin_targets
    FILE delta_kinematics_plugin_targets.cmake
    NAMESPACE kinematics_interface_delta::
    DESTINATION share/${PROJECT_NAME}/cmake
  )
  ament_export_targets(delta_kinematics_plugin_targets HAS_LIBRARY_TARGET)
endif()

ament_package()
