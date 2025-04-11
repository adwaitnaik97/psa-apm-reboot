cmake_minimum_required(VERSION 2.8.3)

include_directories(${CMAKE_CURRENT_LIST_DIR})

## Generate messages from these folders:
add_message_files(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../msg/Input)
add_message_files(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../msg/Output)
add_message_files(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../msg/Misc)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  )
add_library(hg_nav_node_message_support SHARED ${CMAKE_CURRENT_LIST_DIR}/message_support.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1001.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1002.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1003.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1004.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1005.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1101.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1105.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1108.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1111.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1201.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_1401.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2001.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2002.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2005.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2011.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2021.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_20FF.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2201.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2301.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2311.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2401.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2402.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2411.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2421.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2422.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2423.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2424.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2425.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2426.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2427.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2428.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2429.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2501.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_2611.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4012.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4109.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4110.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4201.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4202.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4204.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4401.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4404.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4438.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4651.cpp
	${CMAKE_CURRENT_LIST_DIR}/Input/Msg_4738.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5001.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5012.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5101.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5102.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5103.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5104.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5105.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5106.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5108.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5109.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_5201.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6001.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6003.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6108.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6109.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6110.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6111.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6112.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6201.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6202.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6203.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6204.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6205.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6211.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6311.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6403.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6405.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6406.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6424.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6428.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6438.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6504.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6505.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6508.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6601.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6651.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6721.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6722.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6723.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6724.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6725.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6726.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_6738.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_9900.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_9910.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_CE01.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_FE01.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_FE02.cpp
)
target_link_libraries(hg_nav_node_message_support ${catkin_LIBRARIES} hguide_api)
add_dependencies(hg_nav_node_message_support hg_nav_node_generate_messages_cpp)
message(STATUS,"Custom hg_nav_node ROS Messages Included Successfully")