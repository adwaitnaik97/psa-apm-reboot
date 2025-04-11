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
add_library(hg_node_message_support SHARED ${CMAKE_CURRENT_LIST_DIR}/message_support.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_01.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_02.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_04.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_05.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_0C.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_0D.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_A1.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_A2.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_A3.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_A9.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_AC.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_AD.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_AE.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_B0.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_B1.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_B2.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_B3.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_B4.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_CA.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F0.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F1.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F2.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F3.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F4.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F5.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F6.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F7.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F8.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_F9.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_FA.cpp
	${CMAKE_CURRENT_LIST_DIR}/Output/Msg_FB.cpp
)
target_link_libraries(hg_node_message_support ${catkin_LIBRARIES} hguide_api)
add_dependencies(hg_node_message_support hg_node_generate_messages_cpp)
message(STATUS,"Custom hg_node ROS Messages Included Successfully")