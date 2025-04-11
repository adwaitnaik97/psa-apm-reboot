# Remote Operations
This package implements Paho C++ client library functions to communicate with Remote Console over Mosquitto MQTT broker. 

Cap’n Proto messaging protocol is used as data interchange format.

This package works with remote_handler package. 

 1. Receives AIOS visualization messages from remote_handler by subscribing <strong>/aios/remote_handler/mqtt/to_client</strong> ROS topic.<br>Publishes AIOS visualization messages to MQTT Broker topics.


 2. Subscribes MQTT Broker topics and receives command messages from Remote Console.<br> Publishes remote console command messages to <strong>/aios/remote_handler/mqtt/from_client</strong> ROS topic.
   
   



### Dependencies
https://aidrivers.atlassian.net/wiki/spaces/AIDRIVERS/pages/283049992/MQTT+Setup
```
 Paho C/C++ Libraries
```
```
 C++ Cap’n Proto Library 
```
``` 
 Mosquitto MQTT Broker
```
