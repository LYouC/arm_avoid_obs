cmake_minimum_required(VERSION 3.8)
project(obs_avoid)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(test_moveit src/test_moveit.cpp src/moveit_utils.cpp)
ament_target_dependencies(
  test_moveit
  rclcpp moveit_ros_planning_interface moveit_visual_tools
)
target_include_directories(test_moveit PUBLIC include)

add_executable(avoid_obs src/avoid_obs.cpp src/moveit_utils.cpp src/Obstacle.cpp)
ament_target_dependencies(
  avoid_obs
  rclcpp moveit_ros_planning_interface moveit_visual_tools std_msgs
)
target_include_directories(avoid_obs PUBLIC include)

# add_executable(test_transform test/test_transform.cpp)
# target_link_libraries(test_transform PUBLIC Eigen3::Eigen)
# target_include_directories(test_transform PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)



# install cpp exe file
install(TARGETS
  test_moveit avoid_obs
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
# install python modules
file(GLOB_RECURSE PY_LIST ${PROJECT_NAME}/[a-z]*.py)
install(PROGRAMS
  ${PY_LIST}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

