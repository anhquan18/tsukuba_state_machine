cmake_minimum_required(VERSION 3.8)
project(hfsm2_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# HFSM2 is header-only; fetch headers at configure-time.
include(FetchContent)
FetchContent_Declare(hfsm2
  GIT_REPOSITORY https://github.com/andrew-gresyk/HFSM2.git
  #GIT_TAG        v2.8.0       # pin a release
)
FetchContent_MakeAvailable(hfsm2)

add_executable(hfsm2_demo_node src/hfsm2_demo_node.cpp)
ament_target_dependencies(hfsm2_demo_node rclcpp std_msgs)
target_include_directories(hfsm2_demo_node PRIVATE ${hfsm2_SOURCE_DIR}/include)

install(TARGETS hfsm2_demo_node DESTINATION lib/${PROJECT_NAME})

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

ament_package()

