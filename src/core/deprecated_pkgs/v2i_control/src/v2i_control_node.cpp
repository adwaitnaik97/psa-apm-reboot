#include "v2i_control/v2i_control_node.h"
#include <thread>
#include <mutex>

#include <boost/algorithm/string.hpp>
#include <traffic_light_common/tl_utils.h>
#include <v2i_control/v2i_control.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <jsk_recognition_utils/geo/cube.h>
#include <std_msgs/Bool.h>
#include <aipe_msgs/Classification.h>
#include <ros/package.h>

aipe::V2IControlApp::V2IControlApp() : current_lane_("29.1"),
                                       light_indicator_(1)
{
}

void aipe::V2IControlApp::JunctionAnalysis()
{
    auto start_time_ = std::chrono::high_resolution_clock::now();
    std::string final_state = "";
    float state_conf = 0.0f;
    // Use V2I info if available
    // if (V2I_traffic_light_->Available())
    bool state_valid = V2I_traffic_light_->JunctionState(current_lane_, light_indicator_, junction_id_, final_state, state_conf);

    PublishTLSignal(state_valid, final_state, state_conf);

    auto end_time_ = std::chrono::high_resolution_clock::now();
}

bool aipe::V2IControlApp::PublishTLSignal(bool state_valid, std::string &final_state, float &state_conf)
{
    bool status = true;
    // publish empty containers
    if (!state_valid)
    {
        visualization_msgs::MarkerArray junction_state_marker;
        junction_state_marker.markers.clear();
        pub_junction_marker_.publish(junction_state_marker);
        aipe_msgs::Classification junction_state;
        junction_state.header = msg_header_;
        junction_state.class_index = -1;
        pub_junction_.publish(junction_state);
        return true;
    }

    // tl_management node would ignore this message if last msg's header was used during min_end_time period 
    // since last msg's timestamp is expired
    // Timestamp is updated only when there is a valid state. 
    std_msgs::Header updated_header = msg_header_;
    updated_header.stamp = ros::Time::now();
    
    visualization_msgs::MarkerArray junction_state_marker = aipe::TLUtils::ObjectsToMarkers(final_state, updated_header);
    pub_junction_marker_.publish(junction_state_marker);

    aipe_msgs::Classification junction_state;
    junction_state.header = updated_header;
    junction_state.class_name = final_state;
    boost::algorithm::to_lower(junction_state.class_name);
    junction_state.confidence = state_conf;
    junction_state.class_index = aipe::TLUtils::GetClassIndex(junction_state.class_name);
    pub_junction_.publish(junction_state);
    return status;
}

void aipe::V2IControlApp::V2ICallback(const aios_apm_msgs::V2I::ConstPtr &in_message)
{
    // read V2I message
    if (in_message)
    {
        V2I_traffic_light_->Update(in_message);
        msg_header_ = in_message->header;
        msg_header_.frame_id = output_frame_id_;
    }
}

void aipe::V2IControlApp::LaneCallback(const std_msgs::String::ConstPtr &in_lane)
{
    current_lane_ = in_lane->data;
}

void aipe::V2IControlApp::LightIndicatorCallback(const std_msgs::Int64::ConstPtr &in_light_indicator)
{
    light_indicator_ = in_light_indicator->data;
}

void aipe::V2IControlApp::JunctionIdCallback(const std_msgs::String::ConstPtr in_msg)
{
    if (in_msg)
    {
        junction_id_ = in_msg->data;
    }
}

void aipe::V2IControlApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
    // get params
    std::string tl_control_topic_str = "aipe/tl_signal/v2i";
    std::string tl_control_marker_topic = "aipe/tl_signal/v2i/marker";
    std::string lane_src_topic = "/aidc/run_iterator/current_track_index";
    std::string light_indicator_src_topic = "/aidc/run_iterator/light/indicator";
    std::string v2i_topic = "/v2i/status";
    std::string v2i_dict_file, direction_map_file = "";
    std::string workspace_dir = ros::package::getPath("psa_apm_core_config");

    in_private_handle.param<std::string>("/v2i_control/tl_v2i_topic", tl_control_topic_str, "/aipe/tl_signal/v2i");
    ROS_INFO("[%s] tl_v2i_topic: %s", __APP_NAME__, tl_control_topic_str.c_str());

    in_private_handle.param<std::string>("/v2i_control/lane_src_topic", lane_src_topic, "/aidc/run_iterator/current_track_index");
    ROS_INFO("[%s] lane_src_topic: %s", __APP_NAME__, lane_src_topic.c_str());

    in_private_handle.param<std::string>("/v2i_control/light_indicator_src_topic", light_indicator_src_topic, "/aidc/run_iterator/light/indicator");
    ROS_INFO("[%s] light_indicator_src_topic: %s", __APP_NAME__, light_indicator_src_topic.c_str());

    in_private_handle.param<std::string>("/v2i_control/v2i_topic", v2i_topic, "/v2i/status");
    ROS_INFO("[%s] v2i_topic: %s", __APP_NAME__, v2i_topic.c_str());

    in_private_handle.param<std::string>("/v2i_control/v2i_dictionary", v2i_dict_file, "src/aipe/src/tl_vision_control/src/cfg/v2i.json");
    // v2i_dict_file = workspace_dir + '/' + v2i_dict_file;
    ROS_INFO("[%s] v2i_dictionary_file: %s", __APP_NAME__, v2i_dict_file.c_str());

    in_private_handle.param<std::string>("/v2i_control/direction_map", direction_map_file, "src/aipe/src/tl_vision_control/src/cfg/direction_map.yaml");
    // direction_map_file = workspace_dir + '/' + direction_map_file;
    ROS_INFO("[%s] direction_map_file: %s", __APP_NAME__, direction_map_file.c_str());

    float max_v2i_dist = 50;
    in_private_handle.param<float>("/v2i_control/v2i_max_dist_to_junction", max_v2i_dist, 50);
    ROS_INFO("[%s] v2i_max_dist_to_junction: %f", __APP_NAME__, max_v2i_dist);

    float v2i_time_buffer = 2000;
    in_private_handle.param<float>("/v2i_control/v2i_time_buffer", v2i_time_buffer, 2000);
    ROS_INFO("[%s] v2i_time_buffer: %f", __APP_NAME__, v2i_time_buffer);

    in_private_handle.param<std::string>("/v2i_control/output_frame_id", output_frame_id_, "lidar_link");
    ROS_INFO("[%s] output_frame_id: %s", __APP_NAME__, output_frame_id_.c_str());

    std::string junction_id_topic = "/aipe/junction_id";
    in_private_handle.param<std::string>("/v2i_control/junction_id_topic", junction_id_topic, "/aipe/junction_id");
    ROS_INFO("[%s] junction_id_topic: %s", __APP_NAME__, junction_id_topic.c_str());

    // generate subscribers and sychronizers
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, lane_src_topic.c_str());
    sub_lane_ = in_private_handle.subscribe(lane_src_topic,
                                            1,
                                            &aipe::V2IControlApp::LaneCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, light_indicator_src_topic.c_str());
    sub_light_indicator_ = in_private_handle.subscribe(light_indicator_src_topic,
                                                       1,
                                                       &aipe::V2IControlApp::LightIndicatorCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, v2i_topic.c_str());
    sub_v2i_ = in_private_handle.subscribe(v2i_topic,
                                           1,
                                           &aipe::V2IControlApp::V2ICallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, junction_id_topic.c_str());
    sub_junction_id_ = in_private_handle.subscribe(junction_id_topic,
                                                   1,
                                                   &aipe::V2IControlApp::JunctionIdCallback, this);

    pub_junction_ = node_handle_.advertise<aipe_msgs::Classification>(tl_control_topic_str, 1);
    pub_junction_marker_ = node_handle_.advertise<visualization_msgs::MarkerArray>(tl_control_marker_topic, 1);

    ROS_INFO("[%s] Publishing traffic light control in %s", __APP_NAME__, tl_control_topic_str.c_str());
    ROS_INFO("[%s] Publishing traffic light junction marker in %s", __APP_NAME__, tl_control_marker_topic.c_str());

    V2I_traffic_light_ = boost::shared_ptr<aipe::TrafficLightControlV2I>(new aipe::TrafficLightControlV2I(v2i_dict_file, direction_map_file, max_v2i_dist, v2i_time_buffer));
}

void aipe::V2IControlApp::Run()
{
    ros::NodeHandle private_node_handle("~");
    InitializeRosIo(private_node_handle);
    ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        JunctionAnalysis();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("[%s] END", __APP_NAME__);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, __APP_NAME__);
    aipe::V2IControlApp app;
    app.Run();
    return 0;
}