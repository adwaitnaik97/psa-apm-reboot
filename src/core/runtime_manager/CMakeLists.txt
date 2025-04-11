cmake_minimum_required(VERSION 3.0.2)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  accel_cmd.msg
  steer_cmd.msg
  brake_cmd.msg
  lamp_cmd.msg
  indicator_cmd.msg
  traffic_light.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES runtime_manager
  CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)


catkin_install_python(PROGRAMS
        scripts/aios_runtime_manager_dialog-mapping.py
        scripts/aios_runtime_manager_dialog.py
        scripts/ftrace.py
        scripts/proc_manager.py
        scripts/rtmgr.py
        #scripts/runtime_manager_dialog.py
        scripts/test_pub.py
        scripts/test_srv.py
        scripts/aide_nodes_launch.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
        scripts/3dm_gx5_15.sh
        scripts/add_perm.sh
        scripts/adis16470.sh
        scripts/avt_vimba.sh
        scripts/gazebo.sh
        scripts/mti300.sh
        scripts/vg440.sh
    	#scripts/cleanup
    	#scripts/run
    	#scripts/run_proc_manager
    	#scripts/subnet_chk.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(FILES
        scripts/computing.yaml
       # scripts/data.yaml
        scripts/empty.yaml
        scripts/interface.yaml
        scripts/main.yaml
        scripts/map.yaml
        scripts/qs.yaml
        scripts/rtmgr_icon.xpm
        scripts/rtmgr.wxg
        scripts/sensing.yaml
        scripts/setup.yaml
        scripts/simulation.yaml
        scripts/state.yaml
        scripts/status.yaml
        scripts/topics.yaml
        scripts/vector_map_files.yaml
        scripts/viewer.yaml
        scripts/navigation.yaml

        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        scripts/images/aios_icon.png
        scripts/images/aios_logo_gui.png
        scripts/images/autoware_logo_11.png
        scripts/images/autoware_logo_21.png
        scripts/images/autoware_logo_2.png
        scripts/images/autoware_logo_2_white2.png
        scripts/images/axe.png
        scripts/images/database.png
        scripts/images/dec.png
        scripts/images/inc.png
        scripts/images/mobile.png
        scripts/images/nagoya_university.png
        scripts/images/tablet.png
        scripts/images/vehicle.png
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/images
        #DESTINATION share/${PROJECT_NAME}/images
        )

install(FILES
        scripts/launch_files/check.launch
        scripts/launch_files/control.launch
        scripts/launch_files/init.launch
        scripts/launch_files/map.launch
        scripts/launch_files/perception.launch
        scripts/launch_files/planning.launch
        scripts/launch_files/sensor.launch
        scripts/launch_files/set.launch

        scripts/vscan.launch
        scripts/vehicle_socket.launch
        scripts/uvc_camera.launch
        scripts/traffic_light.launch
        scripts/synchronization.launch
        scripts/tablet_socket.launch
        scripts/setup_tf.launch
        scripts/setup_vehicle_info.launch
        scripts/points2image.launch
        scripts/mqtt_socket.launch
        scripts/ex_mat_pub.launch
        scripts/calibration_publisher.launch
        scripts/avt_camera.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch_files
        #DESTINATION share/${PROJECT_NAME}/launch_files
        )

#add_executable(runtime_manager scripts/runtime_manager.py)
#add_dependencies(runtime_manager runtime_manager_generate_messages_cpp)
