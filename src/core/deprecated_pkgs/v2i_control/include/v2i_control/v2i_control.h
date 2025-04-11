#ifndef PROJECT_V2I_CONTROL_H
#define PROJECT_V2I_CONTROL_H

#define __APP_NAME__ "v2i_control"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <aios_apm_msgs/V2I.h>
#include <chrono>
#include <unordered_map>
#include <geometry_msgs/PoseWithCovariance.h>
#include <traffic_light_common/tl_types.h>

namespace aipe
{
    class TrafficLightControlV2I
    {
    private:

        enum V2IEventState
        {
            UNAVAILABLE = 0,
            DARK = 1,
            FLASHING_RED = 2,
            STEADY_RED = 3,
            PRE_MOVEMENT = 4,
            PERMISSIVE_GREEN = 5,
            PROTECTED_GREEN = 6,
            PERMISSIVE_YELLOW = 7,
            PROTECTED_YELLOW = 8,
            FLASHING_YELLOW = 9
        };
        // >junction_laneID_direction, signal group>
        std::unordered_map<std::string, int> signal_groups_;
        // latest V2I message
        aios_apm_msgs::V2I v2i_msg_;
        bool v2i_received_;
        // max distance(meters) to traffic light to analyze v2i messages
        float max_junction_dist_;
        //time buffer in ms to check received message timestamp. Messages older than the buffer are ignored
        float v2i_time_buffer_;
        // minimum time (in ms) until v2i state to be changed
        int min_end_time_;

    public:
        /*!
        * Final state estimation using the V2I information
        * @param dictionary_path path to file that has signal group mappings
        * @param direction_map_file path to file that has APM direction numbers
        * @param max_junction_dist max distance(meters) to traffic light to analyze v2i messages
        * @param v2i_time_buffer time buffer in ms to check received message timestamp. Messages older than the buffer are ignored
        */
        TrafficLightControlV2I(const std::string &dictionary_path, const std::string &direction_map_file,
                               const float &max_junction_dist, const float &v2i_time_buffer);

        /*!
        * Update current V2I information with the new message
        * @param in_image_msg ros message
        * @return true if updated successfully, false otherwise
        */
        bool Update(const aios_apm_msgs::V2I::ConstPtr &in_msg);

        /*!
        * Load mapping of signal group(received from v2i) for straight move and right turn at each junction and lane.
        * Loaded dictionary is used to find the corresponding signal on the live v2i message
        * @param dictionary_path full path of the dictionary file
        * @param direction_map_file path to file that has APM direction numbers
        * @return true if loaded successfully, false otherwise
        */
        bool LoadSignalGroups(const std::string &dictionary_path, const std::string &direction_map_file);

        /*!
        * Make final state decision according to the predefined rules.
        * @param lane_id current lane id (i.e. 44.2)
        * @param light_indicator light indicator representing the next movement of APM (0: straight, 1: right, 2:left)
        * @param junction_id closest junction id 
        * @param final_state final state received from V2I, -1 if unavailable
        * @param state_conf State confidence, 0.0 to 1.0 
        * @return true if state is determined successfully, false otherwise
        */
        bool JunctionState(const std::string &lane_id, const int light_indicator,
                           std::string &junction_id, std::string &final_state, float &state_conf);

        /*!
        * Get event state of given junction id and signal group
        * @param junction_id current junction id (i.e. CJ1)
        * @param signal_group signal group (1 to 5) representing traffic light group according to the traffic flow direction
        * @return event state(0 to 9), -1 if unavailable
        */
        int GetStateOfSignalGroup(const std::string &junction_id, int signal_group);

        /*!
        * Check if a valid V2I information exists
        * @param final_state final junction state. Availability depends on final state since 
        *                    different min_end_time control is only applied to PROTECTED_GREEN. 
        * @return true if V2I valid, false otherwise
        */
        bool Available(const std::string &final_state);
    };
}

#endif //PROJECT_V2I_CONTROL_H
