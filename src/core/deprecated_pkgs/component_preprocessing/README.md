# AIPE Topic Manager

This node is a bridge between AIPE and its input components. It subscribes to the topics required by AIPE and publishes them isolated from the component-dependent messages. 
Below is the list of input and output topics:
- /aide_info/zones -> /aipe/preprocessing/inside_bay (used by box_filter, scaled_yolo, lidar_detection_filter and chassis_alignment)
- /aide_info/zones -> /aipe/preprocessing/inside_workshop (used by box_filter)
- /aide_info/zones -> /aipe/preprocessing/chassis_lane (used by lane_projection)
- /aisc/situation_cognition/inside_observation_area -> /aipe/preprocessing/inside_observation_area (used by ransac_ground_removal_nodelet, lidar_detection_filter, lidar_euclidean_cluster_detect, chassis_alignment)
- /aisc/situation_cognition/taxi_flag -> /aipe/preprocessing/taxi_flag (used by lane_projection)
- /aide/pose/filtered/utm/lidarlink -> /aipe/preprocessing/pose/filtered/utm/lidarlink (used by roi_filter and radar_roi_filter)
- /aide/odometry/filtered/utm/baselink -> /aipe/preprocessing/odometry/filtered/utm/baselink (used by trailer_localization)
- /airs/plc/critical_fbk/reg_37 -> /aipe/preprocessing/apm_speed (used by sensor_fusion and trailer_localization)

### From a sourced terminal:

`roslaunch component_preprocessing aipe_preprocessing.launch`

### Requirements
* `aide`
* `airs`
* `aisc`

### Parameters

Launch file or lidar_config.yaml available parameters for `chassis_alignment`

|Parameter| Type| Description|
----------|-----|--------
|`inside_bay_topic`|*String*| Original inside bay topic published by aide|
|`inside_bay_topic_remapped`|*String*| Remapped inside bay topic for use of AIPE|
|`parking_area_topic`|*String*| Original workshop area topic published by aide|
|`parking_area_topic_remapped`|*String*| Remapped workshop area topic for use of AIPE|
|`chassis_lane_topic`|*String*| Original chassis lane topic published by aide|
|`chassis_lane_topic_remapped`|*String*| Remapped chassis lane topic for use of AIPE|
|`junction_area_topic`|*String*| Original junction bay topic published by aisc|
|`junction_area_topic_remapped`|*String*| Remapped junction bay topic for use of AIPE|
|`taxi_flag_topic`|*String*| Original taxi flag topic published by aisc|
|`taxi_flag_topic_remapped`|*String*| Remapped taxi flag topic for use of AIPE|
|`apm_position_lidarlink_topic`|*String*| Original truck position (lidarlink) topic published by aide|
|`apm_position_lidarlink_topic_remapped`|*String*| Remapped truck position (lidarlink) topic for use of AIPE|
|`apm_position_lidarlink_topic`|*String*| Original truck position (baselink) topic published by aide|
|`apm_position_lidarlink_topic_remapped`|*String*| Remapped truck position (baselink) topic for use of AIPE|
|`apm_position_baselink_topic`|*String*| Original truck position (baselink) topic published by aide|
|`apm_position_baselink_topic_remapped`|*String*| Remapped truck position (baselink) topic for use of AIPE|
|`apm_speed_topic`|*String*| Original truck speed and steering topic published by aide|
|`apm_speed_topic_remapped`|*String*| Remapped truck speed and steering topic for use of AIPE|

### Subscribed topics

|Topic|Type|Objective|Publisher Node
------|----|---------|----------
|`/aide_info/zones`|`aide_apm_msgs::ZoneInfo`|Zones.|aide|
|`/aisc/situation_cognition/inside_observation_area`|`std_msgs::Bool`|Junction area flag.|aisc|
|`/aisc/situation_cognition/taxi_flag`|`std_msgs::Bool`|Taxi flag.|aisc|
|`/aide/pose/filtered/utm/lidarlink`|`nav_msgs::Odometry`|APM position (lidarlink frame).|aide|
|`/aide/odometry/filtered/utm/baselink`|`sensor_msgs::CameraInfo`|APM position (baselink frame).|aide|
|`/airs/plc/critical_fbk/reg_37`|`airs_msgs::reg37`|APM speed and steering angle.|airs|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/aipe/preprocessing/inside_bay`|`std_msgs::Bool`|Flag indicates if APM is inside a bay area|
|`/aipe/preprocessing/inside_workshop`|`std_msgs::Bool`|Flag indicates if APM is inside workshop|
|`/aipe/preprocessing/chassis_lane`|`std_msgs::Bool`|Flag indicates if APM is in a chassis lane|
|`/aipe/preprocessing/inside_observation_area`|`std_msgs::Bool`|Flag indicates if APM is at a TL junction|
|`/aipe/preprocessing/taxi_flag`|`std_msgs::Bool`|Taxi flag|
|`/aipe/preprocessing/pose/filtered/utm/lidarlink`|`nav_msgs::Odometry`|APM position (baselink frame)|
|`/aipe/preprocessing/odometry/filtered/utm/baselink`|`nav_msgs::Odometry`|APM position (baselink frame)|
|`/aipe/preprocessing/apm_speed`|`aipe_msgs::TruckSpeedSteering`|APM speed and steering angle|
