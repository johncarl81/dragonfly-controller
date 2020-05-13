cmake_minimum_required(VERSION 3.0.2)
project(dragonfly_messages_java)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED genjava)

generate_rosjava_messages(
    PACKAGES
    dragonfly_messages
)

catkin_package()


