#include "v2i_control/v2i_control.h"
#include <chrono>
#include <thread>
#include <mutex>

#include <boost/algorithm/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <traffic_light_common/tl_utils.h>
#include <traffic_light_common/tl_types.h>
#include <std_msgs/Bool.h>

aipe::TrafficLightControlV2I::TrafficLightControlV2I(const std::string &dictionary_path,
                                                     const std::string &direction_map_file,
                                                     const float &max_junction_dist,
                                                     const float &v2i_time_buffer) : v2i_received_(false),
                                                                                     max_junction_dist_(max_junction_dist),
                                                                                     v2i_time_buffer_(v2i_time_buffer),
                                                                                     min_end_time_(v2i_time_buffer)
{
    LoadSignalGroups(dictionary_path, direction_map_file);
}

bool aipe::TrafficLightControlV2I::LoadSignalGroups(const std::string &dictionary_path, const std::string &direction_map_file)
{
    std::unordered_map<std::string, int> direction_map;
    aipe::TLUtils::LoadDirectionMap(direction_map_file, direction_map);
    try
    {
        // Load signal group and lane-direction map
        std::ifstream v2i_dict(dictionary_path);
        if (v2i_dict.good())
        {
            boost::property_tree::ptree pt;
            boost::property_tree::read_json(v2i_dict, pt);

            using boost::property_tree::ptree;
            // iterate junctions
            for (ptree::const_iterator it = pt.begin(); it != pt.end(); ++it)
            {
                std::string junction_name = it->first;
                // iterate lanes
                for (ptree::const_iterator it_lane = it->second.begin(); it_lane != it->second.end(); ++it_lane)
                {
                    std::string lane_id = it_lane->first;
                    // std::string map_key = junction_name + '_' + lane_id;
                    //  iterate directions
                    for (ptree::const_iterator it_direction = it_lane->second.begin(); it_direction != it_lane->second.end(); ++it_direction)
                    {
                        std::string direction = it_direction->first;
                        std::string map_key = lane_id + '_' + std::to_string(direction_map[direction]);
                        // iterate signal group ids
                        for (ptree::const_iterator it_tls = it_direction->second.begin(); it_tls != it_direction->second.end(); ++it_tls)
                        {
                            std::string tl_id_str = it_tls->second.get_value<std::string>();
                            int tl_id = std::atoi(tl_id_str.c_str());
                            // assumed there is only one signal_group for a junction_lane_direction key
                            signal_groups_[map_key] = tl_id;
                            std::cout << "Loaded signal: " << map_key << " - " << tl_id << std::endl;
                        }
                    }
                }
            }
            ROS_INFO("[%s] Signal groups loaded successfully!", __APP_NAME__);
        }
        else
        {
            ROS_ERROR("[%s] Could not load v2i file: %s!", __APP_NAME__, dictionary_path.c_str());
        }
    }
    catch (std::exception const &e)
    {
        ROS_ERROR("[%s] Error while loading v2i file!", __APP_NAME__);
        std::cerr << e.what() << std::endl;
    }
    return true;
}

bool aipe::TrafficLightControlV2I::Update(const aios_apm_msgs::V2I::ConstPtr &in_msg)
{
    if (in_msg)
    {
        v2i_msg_ = *in_msg;
        v2i_received_ = true;
    }
}

int aipe::TrafficLightControlV2I::GetStateOfSignalGroup(const std::string &junction_id, int signal_group)
{
    int state = -1;

    // find the junction id in the intersections of last v2i message
    for (auto junction : v2i_msg_.intersections)
    {
        bool found = false;
        // check if junction traffic light system is working properly (status == 6)
        if (junction.status == 6)
        {
            if (junction.intersection_id == junction_id)
            {
                // find signal group of the junction
                for (auto sig_gr : junction.states)
                {
                    if (sig_gr.signal_group == signal_group)
                    {
                        state = sig_gr.event_state;
                        min_end_time_ = sig_gr.min_end_time;
                        found = true;
                        break;
                    }
                }
            }
        }
        else
        {
            // return -8 for traffic light failure mode
            // return -9 for traffic light controller powered down error
            state = 0 - junction.status;
            min_end_time_ = -1;
        }
        if (found)
        {
            break;
        }
    }

    return state;
}

bool aipe::TrafficLightControlV2I::JunctionState(const std::string &lane_id, const int light_indicator,
                                                 std::string &junction_id, std::string &final_state, float &state_conf)
{
    bool status = true;
    // // do not generate traffic light signal if APM's distance to junction is more than 50 meters
    // if (junction_dist > max_junction_dist_)
    // {
    //     final_state = "";
    //     state_conf = 0.5;
    //     return false;
    // }

    // signal group ids are stored for each lane and light indicator
    std::string signal_key = lane_id + '_' + std::to_string(light_indicator);
    if (signal_groups_.find(signal_key) != signal_groups_.end())
    {
        int signal_group = signal_groups_[signal_key];
        // get traffic light state of the related signal group
        int state = GetStateOfSignalGroup(junction_id, signal_group);
        if (state == (int)V2IEventState::PROTECTED_GREEN)
        {
            final_state = "Green";
        }
        else if (state < 0)
        {
            status = false;
        }
        /*else if(state == (int)V2IEventState::UNAVAILABLE || state == (int)V2IEventState::DARK)  // v2i error modes
        {
            status = false;
        }*/
        else
        {
            final_state = "Red";
        }
        if (final_state == "Green" && light_indicator == 1)
        {
            final_state = "right_arrow";
        }
        state_conf = 1.0f;
    }
    else
    {
        state_conf = 0.0f;
        ROS_INFO("[%s] Given lane id and light indicator combination does not exist in signal group map: %s and %d", __APP_NAME__, lane_id.c_str(), light_indicator);
        status = false;
    }

    if (!Available(final_state))
    {
        final_state = "";
        status = false;
    }
    return status;
}

bool aipe::TrafficLightControlV2I::Available(const std::string &final_state)
{
    // If any v2i messages haven't been received yet
    if (!v2i_received_)
    {
        return false;
    }

    // check if last v2i message is not expired
    std::time_t cur_time = ros::Time::now().toNSec() * 10e-7;
    int msg_age = std::abs(cur_time - v2i_msg_.timestamp);

    if (final_state == "Green" || final_state == "right_arrow")
    {
        //std::cout << "msg_age           : " << msg_age << std::endl;
        //std::cout << "min_end_time_     : " << min_end_time_ << std::endl;

        // Each v2i intersection state has a min_end_time to provide a rough estimation of how long the current state will remain.
        // Use min_end_time when state is PROTECTED_GREEN to keep publishing current state even there is no valid msg received
        if (min_end_time_ != -1 && msg_age < min_end_time_ - 2000)
        {
            return true;
        }
        else
        {
            min_end_time_ = -1;
        }
    }
    if (std::abs(msg_age) < v2i_time_buffer_)
    {
        return true;
    }
    return false;
}
