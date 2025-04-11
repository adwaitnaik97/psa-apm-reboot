#ifndef PROJECT_V2I_CONTROL_NODE_H
#define PROJECT_V2I_CONTROL_NODE_H

#define __APP_NAME__ "v2i_control"

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <aipe_msgs/DetectedObjectArray.h>
#include <aios_apm_msgs/V2I.h>
#include <v2i_control/v2i_control.h>

#include <traffic_light_common/tl_types.h>
#include <chrono>

namespace aipe
{
    class V2IControlApp
    {

    public:
        void Run();

        /*!
        * ROS Node to parse received V2I message and find junction traffic light state considering current lane id and light indicator
        * 
        * Traffic light rules for each junction and lane, utm coordinates of traffic light poles are loaded to determine the state of the junction.
        * Status of all traffic lights belonging to the current junction is being published with their utm coordinates. 
        */
        V2IControlApp();

        /*!
        * Reads the config params from the command line
        * @param in_private_handle
        */
        void InitializeRosIo(ros::NodeHandle &in_private_handle);

        /*!
        * Callback to get current lane of APM. Used in decision of junction status regarding the traffic lights
        * @param in_lane ros message
        */
        void LaneCallback(const std_msgs::String::ConstPtr &in_lane);

        /*!
        * Callback to get light indicator representing the next movement of APM
        * @param in_light_indicator ros message
        */
        void LightIndicatorCallback(const std_msgs::Int64::ConstPtr &in_light_indicator);

        /*!
        * Callback to get V2I junction and ARMG messages
        * @param in_message ros message
        */
        void V2ICallback(const aios_apm_msgs::V2I::ConstPtr &in_message);

        /*!
        * Callback to get junction id (ppt09, ppt10, etc.)
        * @param in_message ros message
        */
        void JunctionIdCallback(const std_msgs::String::ConstPtr in_msg);

        /*!
        * Find traffic light state of the junction
        */
        void JunctionAnalysis();

        /*!
        * Publish traffic light signal 
        * @param state_valid flag if a valid state is available
        * @param final_state final traffic light state
        * @param state_conf final state confidence (0 to 1)
        */
        bool PublishTLSignal(bool state_valid, std::string &final_state, float &state_conf);

    private:
        // Node handle and publisher
        ros::NodeHandle node_handle_;
        ros::Publisher pub_junction_;
        ros::Publisher pub_junction_marker_;
        ros::Publisher pub_traffic_lights_;
        ros::Publisher pub_tl_markers_;

        // Subscribers
        ros::Subscriber sub_lane_;
        // representing estimated movement of APM-> 0:moving straight, 1: turn right, 2:turn left, 3:based on apm behaviour
        ros::Subscriber sub_light_indicator_;
        ros::Subscriber sub_v2i_;
        ros::Subscriber sub_junction_id_;

        // APM pose
        geometry_msgs::PoseWithCovariance apm_pose_;
        // lane index of APM received from the laneCallback (format:"0.0")
        std::string current_lane_;
        // light_indicator representing movement of APM (0: moving straight, 1:turn right, 2:turn left, 3:based on apm behaviour)
        int light_indicator_;
        // V2I junction traffic light integration
        boost::shared_ptr<aipe::TrafficLightControlV2I> V2I_traffic_light_;
        // Junction id (ppt09, ppt10, etc)
        std::string junction_id_;
        // output message header
        std_msgs::Header msg_header_;
        std::string output_frame_id_;

    };
}

#endif //PROJECT_V2I_CONTROL_NODE_H
