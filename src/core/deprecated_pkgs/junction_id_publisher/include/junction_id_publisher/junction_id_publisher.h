#ifndef PROJECT_JUNCTION_ID_PUBLISHER_NODE_H
#define PROJECT_JUNCTION_ID_PUBLISHER_NODE_H

#define __APP_NAME__ "junction_id_publisher"

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <junction_id_publisher/junction_id_publisher.h>

#include <traffic_light_common/tl_types.h>
#include <chrono>

namespace aipe
{
    class JunctionIdPublisher
    {

    public:
        void Run();

        /*!
        * ROS Node to publish junction id (i.e. ppt09, ppt10)
        * 
        * Traffic light utm coordinates are loaded from a config file and APM's currenct position is used to find 
        * closest junction.
        */
        JunctionIdPublisher();

        /*!
        * Reads the config params from the command line
        * @param in_private_handle
        */
        void InitializeRosIo(ros::NodeHandle &in_private_handle);

        /*!
        * Callback to get APM pose
        * @param in_message ros message
        */
        void OdomCallback(const nav_msgs::Odometry::ConstPtr in_msg);
        
        /*!
        * Publish junction id for v2i communication (i.e. PPT09, PPT10, etc.)
        */
        void PublishJunctionId();

    private:
        // Node handle and publisher
        ros::NodeHandle node_handle_;
        ros::Publisher pub_junction_id_;

        // Subscribers
        ros::Subscriber sub_odom_;

        // UTM coordinates of traffic light poles
        std::vector<aipe::traffic_light_common::TrafficPole> tl_poles_;
        // file path including the traffic light pole coordinates
        std::string tl_poles_file_;

        // APM pose
        geometry_msgs::PoseWithCovariance apm_pose_;
        // lane index of APM received from the laneCallback (format:"0.0")
        std::string current_lane_;
        // light_indicator representing movement of APM (0: moving straight, 1:turn right, 2:turn left, 3:based on apm behaviour)
        int light_indicator_;
    };
}

#endif //PROJECT_JUNCTION_ID_PUBLISHER_NODE_H
