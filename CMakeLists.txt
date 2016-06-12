cmake_minimum_required(VERSION 2.8.3)
project(tf_to_imu)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS tf2 tf2_ros)

catkin_package(
)

add_definitions("-std=c++0x -Wall -Werror")

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_to_imu src/tf_to_imu.cpp)
target_link_libraries(tf_to_imu ${catkin_LIBRARIES})
add_dependencies(tf_to_imu ${catkin_EXPORTED_TARGETS})

add_executable(imu_to_tf src/imu_to_tf.cpp)
target_link_libraries(imu_to_tf ${catkin_LIBRARIES})
add_dependencies(imu_to_tf ${catkin_EXPORTED_TARGETS})
