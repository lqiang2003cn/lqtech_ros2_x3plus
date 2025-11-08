cmake_minimum_required(VERSION 3.16)
project(lqtech_ros2_x3plus LANGUAGES CXX)

message("lqtech_ros2_x3plus project source dir: ${PROJECT_SOURCE_DIR}")     

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
set(HARDWARE_LIB_NAME "lqtech_x3plus_hardware_lib")


# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

######################################## compile ########################################
# source file of the lib

add_library(${HARDWARE_LIB_NAME} SHARED hardware/x3plus.cpp)
# include file of the lib
target_compile_features(${HARDWARE_LIB_NAME} PUBLIC cxx_std_17)
target_include_directories(${HARDWARE_LIB_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include> # used by the compiler, not by the ide, this must be the absolute path
$<INSTALL_INTERFACE:include/${PROJECT_NAME}> # relative path. tells users to how to find the include folder, should be consistent with DESTINATION include/lqtech_ros2_x3plus
)
# dependencies of the lib
ament_target_dependencies(
  ${HARDWARE_LIB_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)



######################################## install ########################################
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${HARDWARE_LIB_NAME}
  EXPORT export_${HARDWARE_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)

#   ament_add_pytest_test(example_1_urdf_xacro test/test_urdf_xacro.py)
#   ament_add_pytest_test(view_example_1_launch test/test_view_robot_launch.py)
#   ament_add_pytest_test(run_example_1_launch test/test_rrbot_launch.py)
#   ament_add_pytest_test(run_example_1_launch_cli_direct test/test_rrbot_launch_cli_direct.py)
# endif()


## EXPORTS
ament_export_targets(export_${HARDWARE_LIB_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
