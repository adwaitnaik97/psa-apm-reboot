#include "junction_id_publisher/junction_id_publisher.h"
#include <thread>
#include <mutex>

#include <boost/algorithm/string.hpp>
#include <traffic_light_common/tl_utils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <jsk_recognition_utils/geo/cube.h>
#include <std_msgs/Bool.h>
#include <aipe_msgs/Classification.h>
#include <ros/package.h>

aipe::JunctionIdPublisher::JunctionIdPublisher()
{
}

void aipe::JunctionIdPublisher::PublishJunctionId()
{
    auto start_time_ = std::chrono::high_resolution_clock::now();

    // get junction id using APM pose
    aipe::traffic_light_common::TrafficPole closest_pole;
    float junction_dist = 0;
    if (!aipe::TLUtils::FindClosestJunction(apm_pose_, tl_poles_, closest_pole, junction_dist))
    {
        closest_pole.m_junction = "";
    }

    std_msgs::String junc_msg;
    junc_msg.data = closest_pole.m_junction;
    pub_junction_id_.publish(junc_msg);

    auto end_time_ = std::chrono::high_resolution_clock::now();
}

void aipe::JunctionIdPublisher::OdomCallback(const nav_msgs::Odometry::ConstPtr in_msg)
{
    if (in_msg)
    {
        apm_pose_ = in_msg->pose;
    }
}

void aipe::JunctionIdPublisher::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
    //get params
    std::string odom_topic = "/aide/odometry/filtered/utm/baselink";
    std::string lanes_file, v2i_dict_file, direction_map_file, poles_position_file = "";
    std::string workspace_dir = ros::package::getPath("psa_apm_core_config");

    in_private_handle.param<std::string>("/v2i_control/odom_topic", odom_topic, "/aide/odometry/filtered/utm/baselink");
    ROS_INFO("[%s] odom_topic: %s", __APP_NAME__, odom_topic.c_str());

    in_private_handle.param<std::string>("/junction_id_publisher/traffic_lights_file", tl_poles_file_, "src/aipe/src/tl_vision_control/src/cfg/traffic_lights.json");
    tl_poles_file_ = workspace_dir + '/' + tl_poles_file_;
    ROS_INFO("[%s] traffic_lights_file: %s", __APP_NAME__, tl_poles_file_.c_str());

    std::string junction_id_topic = "/aipe/junction_id";
    in_private_handle.param<std::string>("/v2i_control/junction_id_topic", junction_id_topic, "/aipe/junction_id");
    ROS_INFO("[%s] junction_id_topic: %s", __APP_NAME__, junction_id_topic);

    //generate subscribers and sychronizers
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, odom_topic.c_str());
    sub_odom_ = in_private_handle.subscribe(odom_topic,
                                            1,
                                            &aipe::JunctionIdPublisher::OdomCallback, this);

    pub_junction_id_ = node_handle_.advertise<std_msgs::String>(junction_id_topic, 5);

    aipe::TLUtils::LoadTrafficLightPoles(tl_poles_file_, tl_poles_);

    ROS_INFO("[%s] Publishing junction id in %s", __APP_NAME__, junction_id_topic.c_str());
}

void aipe::JunctionIdPublisher::Run()
{
    ros::NodeHandle private_node_handle("~");
    InitializeRosIo(private_node_handle);
    ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        PublishJunctionId();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("[%s] END", __APP_NAME__);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, __APP_NAME__);
    aipe::JunctionIdPublisher app;
    app.Run();
    return 0;
}