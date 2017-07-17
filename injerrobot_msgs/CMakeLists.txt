cmake_minimum_required(VERSION 2.8.3)
project(injerrobot_msgs)
find_package(catkin REQUIRED COMPONENTS
   message_generation
   std_msgs
   actionlib_msgs
   actionlib
)

###add_message_files(
###   FILES
###   ChairState.msg
###   IsupportState.msg
###)
###
###add_service_files(
###   FILES
###   QueryState.srv
###)

add_action_files(
   DIRECTORY action
   FILES Operation.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
