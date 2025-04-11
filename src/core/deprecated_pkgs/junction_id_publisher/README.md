# Junction ID Publisher

ROS Node to use APM position to find closest junction's id. List of traffic light poles with UTM coordinates are loaded to compare the position with APM's pose.

### From a sourced terminal:

`roslaunch junction_id_publisher junction_id_publisher.launch`

## Requirements

* `traffic_lights.json` file (See parameters section for the file location)
* `aide`

## Parameters

Config file({projectname_core_config}) available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`odom_topic`|*String* |To get APM's global position.|`/aide/odometry/filtered/utm/baselink`|
|`traffic_lights_file`|*String* |To load traffic light poles' global coordinates|`{projectname_core_config}/traffic_light_management/cfg/traffic_lights.json`|
|`junction_id_topic`|*String* |Output topic to publish junction ID (i.e. ppt09, ppt10, etc)|`/aipe/junction_id`|

### Subscribed topics
 
|Topic|Type|Objective|Publisher Node
------|----|---------|-----------
|`/aide/odometry/filtered/utm/baselink`|`nav_msgs::Odometry`|APM's global position.|aide|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/aipe/junction_id`|`std_msgs::String`|Junction ID (i.e. ppt09, ppt10)|

## Example of usage


## Notes

