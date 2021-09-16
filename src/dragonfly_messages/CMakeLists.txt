cmake_minimum_required(VERSION 3.5)
project(dragonfly_messages)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Boundary.msg"
  "msg/CO2GradientFollowing.msg"
  "msg/Curtain.msg"
  "msg/DDSAMission.msg"
  "msg/FlockMission.msg"
  "msg/GotoWaypoint.msg"
  "msg/LatLon.msg"
  "msg/LawnmowerMission.msg"
  "msg/MissionStep.msg"
  "msg/NavigationMission.msg"
  "msg/Semaphore.msg"
  "msg/SemaphoreToken.msg"
  "msg/Sleep.msg"
  "msg/Takeoff.msg"
  "msg/Waypoint.msg"
  "srv/DDSA.srv"
  "srv/DDSAWaypoints.srv"
  "srv/Flock.srv"
  "srv/Lawnmower.srv"
  "srv/LawnmowerWaypoints.srv"
  "srv/Mission.srv"
  "srv/Navigation.srv"
  "srv/Setup.srv"
  "srv/Simple.srv"
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()