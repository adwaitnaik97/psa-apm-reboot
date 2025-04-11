# V2I Control

ROS Node to determine signal group using the lane id and light indicator and get related signal group status from the received v2i message and publish as a junction traffic light state. 
Each traffic flow direction in a junction corresponds to a signal group in the v2i message. 

### From a sourced terminal:

`roslaunch v2i_control v2i_control.launch`

## Requirements

* `aidc` for light indicator and lane id 
* `v2i parser` for signal groups 

## Parameters

Launch file available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`lane_src_topic`|*String* |Topic to get current lane id.|`/aidc/run_iterator/current_track_index`|
|`light_indicator_src_topic`|*String* |Topic to get the light indicator.|`/aidc/run_iterator/light/indicator`|
|`junction_id_topic`|*String* |Topic to get current junction id.|`/aipe/junction_id`|
|`v2i_topic`|*String* |Topic to get v2i message for signal groups|`/v2i/status`|
|`tl_v2i_topic`|*String* |Output topic to publish junction state|`/aipe/tl_signal/v2i`|
|`v2i_dictionary`|*String*|Dictionary mapping lane id and light indicator to a signal group.|`{projectname}_core_config)/traffic_light_management/cfg/v2i.json`|
|`v2i_max_dist_to_junction`|*Float*|Distance to junction in meters to start publishing v2i tl state.|`100`|
|`v2i_time_buffer`|*Float*|Maximum allowed time in milliseconds between to consecutive v2i messages. v2i communication is considered lost in case of delay longer than this buffer|`2000`|
|`output_frame_id_`|*String*|Output frame id.|`lidar_link`|

### Subscribed topics
 
|Topic|Type|Objective|Publisher Node
------|----|---------|-----------
|`/aidc/run_iterator/current_track_index`|`std_msgs::String`|APM lane id|aidc|
|`/aidc/run_iterator/light/indicator`|`std_msgs::Int64`|APM light indicator|aidc|
|`/aipe/junction_id`|`std_msgs::Int64`|Junction id (i.e. ppt09)|junction_id_publisher|
|`/v2i/status`|`std_msgs::String`|v2i message containing signal groups|v2i_tl_client|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/aipe/tl_signal/v2i`|`aipe_msgs::Classification`|Junction signal (0:"red", 1:"amber", 2:"green", 3:"right_arrow", 4:"cas_up", 5:"cas_down")|

## Example of usage

## Notes

