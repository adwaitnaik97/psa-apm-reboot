# Traffic Light Vision Control

ROS Node to determine and publish CAS or junction traffic light state using the cameras.
         
APM's lane id, light indicator, junction hazard, trip job status, top centre/bottom left/bottom right camera classifications, fused objects are used as input. Traffic light rules for each lane are loaded to determine the state of the junction/CAS TL considering all this information. Apart from the junction state, status  of all traffic lights belonging to the current junction is being published with their UTM coordinates.

CAS: Red signal is published regardless of the CAS traffic light when CAS alignment duration reaches the max allowed time (max_CAS_duration param)

### From a sourced terminal:

`roslaunch tl_vision_control tl_vision_control.launch`

## Requirements

* `euclidean_cluster_detect` node.
* `imm_ukf_pda_track` node.
* `scaled_yolo` node.
* `traffic light classifier` node.
* `sensor_fusion` node.
* `aifo`
* `aidc`

## Parameters

Launch file available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`fused_objects`|*String* |Name of the `aipe_msgs::DetectedObjectArray` topic to subscribe containing the fused objects.|`/aipe/fused/tracked_objects`|
|`classifications_fbc`|*String* |Name of the `aipe_msgs::ClassificationArray` topic to subscribe classifications of front top centre camera.|`/aipe/camera_top_center_front/classifications`|
|`classifications_fbr`|*String* |Name of the `aipe_msgs::ClassificationArray` topic to subscribe classifications of front bottom right camera.|`/aipe/camera_top_right_front/classifications`|
|`classifications_fbl`|*String* |Name of the `aipe_msgs::ClassificationArray` topic to subscribe classifications of front bottom left camera.|`/aipe/camera_top_left_front/classifications`|
|`apm_speed_src`|*String*|Name of the `reg37` topic to subscribe containing APM speed.|`/airs/plc/critical_fbk/reg_37`|
|`lane_src_topic`|*String*|Name of the `std_msgs::String` topic that contains the lane id of APM.|`/aidc/run_iterator/current_track_index`|
|`light_indicator_src_topic`|*String*|Name of the `std_msgs::Int64` topic that contains light indicator representing next move of APM.|`/aidc/run_iterator/light/indicator`|
|`hazard_topic`|*String*|Name of the `aipe_msgs::SituationCognition` topic that contains junction hazard situation.|`/aisc/situation_cognition/result`|
|`trip_job_topic`|*String*|Name of the `/aifo_client/TripJobStatus` topic that contains target location id (L0, L1, etc.).|`/aifo/trip_job/status`|
|`apm_status_topic`|*String*|Name of the `std_msgs::Int64` topic that contains apm status (0, 1, ..).|`/aidc/apm_status/status`|
|`lanes_file`|*String*|Json file containing traffic light rules for each lane of junctions.|`cfg/lanes.json`|
|`output_frame_id`|*String*|Frame id to transform all output objects.|`lidar_link`|
|`overlap_threshold`|*float*|A number between 0.1 and 1.0 representing the area of overlap between the detections.|`0.5`|
|`virtual_poles`|*bool*|A virtual pole is created for each real pole to increase the accuracy of the estimation. Virtual poles are used to handle false detection/classification.|`true`|
|`flickering_duration`|*int*|Max allowed duration of a flickering period in frames. Invisibility of cas_up or cas_down less than this period is tolerated and keep publishing last state.|`10`|
|`red_tolerance`|*int*|Less than this number of consecutive red detections are ignored. Latest status is kept publishing during this period.|`3`|
|`max_CAS_duration`|*int*|Maximum allowed time (secs) for a CAS alignment job. Red signal is started to publish once reached|`50`|
|`turn_right_wait_time`|*int*|Number of frames to wait after detecting straight green to turn right at CJ1. Safety buffer for vehicles waiting on the opposite lane to enter the junction|`20`|
|`precision_alignment_thresh`|*int*|Precision alignment flag is published true if any red detection is shown for the number of frames indicated by this threshold. It shows that APM is close to the final destination. |`10`|


### Subscribed topics
 
|Topic|Type|Objective|Publisher Node
------|----|---------|-----------
|`/aipe/fused/tracked_objects`|`aipe_msgs::DetectedObjectArray`|Fused 3D objects.|sensor_fusion|
|`/aipe/camera_top_center_front/classifications`|`aipe_msgs::ClassificationArray`|Classification results of front top centre camera|trafficlight_classifier|
|`/aipe/camera_top_left_front/classifications`|`aipe_msgs::ClassificationArray`|Classification results of front bottom left camera|trafficlight_classifier|
|`/aipe/camera_top_right_front/classifications`|`aipe_msgs::ClassificationArray`|Classification results of front bottom right camera|trafficlight_classifier|
|`/aidc/run_iterator/current_track_index`|`std_msgs::String`|APM lane id|aidc|
|`/aidc/run_iterator/light/indicator`|`std_msgs::Int64`|APM light indicator|aidc|
|`/aipe/radar_tracks`|`aipe_msgs::SituationCognition`|Junction hazard|situation_cognition|
|`/airs/plc/critical_fbk/reg_37`|`airs_msgs/reg37`|APM speed|plc_regVal|
|`/aifo/trip_job/status`|`aifo_client/TripJobStatus`|Trip job status. Required for selection of side camera for CAS|aifo|
|`/aidc/apm_status/status`|`std_msgs::Int64`|Apm status required to count max allowed CAS alignment time|aidc|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/aipe/traffic_light/signal_tl`|`aipe_msgs::Classification`|Junction or CAS signal (0:"red", 1:"amber", 2:"green", 3:"right_arrow", 4:"cas_up", 5:"cas_down")|
|`/aipe/tl_signal/cas`|`aipe_msgs::Classification`|CAS signal (0:"red", 1:"amber", 2:"green", 3:"right_arrow", 4:"cas_up", 5:"cas_down")|
|`/aipe/tl_signal/junction`|`aipe_msgs::Classification`|Junction signal (0:"red", 1:"amber", 2:"green", 3:"right_arrow", 4:"cas_up", 5:"cas_down")|
|`/aipe/traffic_light/signal_tl/marker`|`vizualization_msgs::MarkerArray`|Junction status marker|
|`/aipe/traffic_lights/markers`|`vizualization_msgs::MarkerArray`|Traffic light markers|
|`/aipe/precision_alignment_tl`|`std_msgs::Bool`|CAS precision alignment flag to guide AIDC about the distance to final destination|

### Input Topics

1. Fused objects from Sensor Fusion (`aipe_msgs/DetectedObjectArray`)
2. Classification results of front top centre camera (`aipe_msgs/ClassificationArray`)
3. Classification results of front bottom right camera (`aipe_msgs/ClassificationArray`)
4. Classification results of front bottom left camera (`aipe_msgs/ClassificationArray`)
5. APM lane id (`std_msgs::String`)
6. APM light indicator (`std_msgs::Int64`)
7. Junction hazard (`aipe_msgs::SituationCognition`)
8. APM speed (`airs_msgs/reg37`)
9. Trip job status (`aifo_client/TripJobStatus`)
10. APM status (`std_msgs::Int64`)

## Example of usage

1. Launch Sensor fusion node (Follow instructions on Sensor fusion's README)
2. Launch this node.
3. Launch `rviz`, and add the topics shown above in the Output section.

## Notes

