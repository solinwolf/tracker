cmake_minimum_required(VERSION 2.8.3)
project(subscriber)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)

add_message_files(FILES Distance.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber subscriber_generate_messages_cpp)
