#include "tl_vision_control/tl_vision_control.h"
#include <chrono>
#include <thread>
#include <mutex>

#include <boost/algorithm/string.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_utils/geo/cube.h>

#include "traffic_light_common/tl_utils.h"
#include "tl_vision_control/junction_state_estimator.h"
#include "tl_vision_control/CAS_state_estimator.h"
#include <ros/package.h>

#include <std_msgs/Bool.h>

aipe::TrafficLightControlApp::TrafficLightControlApp() : apm_status_(-1),
                                                         max_CAS_duration_(50),
                                                         precision_alignment_thresh_(1),
                                                         CAS_tl_x_coordinate_(-1),
                                                         left_cam_width_(2048),
                                                         right_cam_width_(2048),
                                                         top_cam_width_(2048),
                                                         junction_id_("")
{
    current_lane_ = "29.1";
    light_indicator_ = 1;
    junction_hazard_ = true;
    inside_bay_ = false;
    target_location_id_ = "";

    classifications_tbc_ = boost::shared_ptr<aipe_msgs::ClassificationArray>(new aipe_msgs::ClassificationArray);
    classifications_tbl_ = boost::shared_ptr<aipe_msgs::ClassificationArray>(new aipe_msgs::ClassificationArray);
    classifications_tbr_ = boost::shared_ptr<aipe_msgs::ClassificationArray>(new aipe_msgs::ClassificationArray);
    classifications_tbr_raw_ = boost::shared_ptr<aipe_msgs::ClassificationArray>(new aipe_msgs::ClassificationArray);
    tbc_time_ = std::chrono::high_resolution_clock::now();
    tbl_time_ = std::chrono::high_resolution_clock::now();
    tbr_time_ = std::chrono::high_resolution_clock::now();

    // list of valid traffic light status for CAS
    CAS_color_list_.push_back("Amber");
    CAS_color_list_.push_back("Red");
    CAS_color_list_.push_back("Cas_up");
    CAS_color_list_.push_back("Cas_down");
}

visualization_msgs::MarkerArray aipe::TrafficLightControlApp::ObjectsToMarkers(const aipe_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray final_markers;

    int i = 0;
    for (const aipe_msgs::DetectedObject &object : in_objects.objects)
    {
        if (!(object.label == "Traffic_light" || object.label == "Right_traffic_light") || (object.pose.position.x == 0 && object.pose.position.y == 0 && object.pose.position.z == 0))
            continue;
        {
            visualization_msgs::Marker marker;
            marker.header = object.header;
            marker.ns = "sensor_fusion";
            marker.id = object.id;
            //marker.id = i;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            //marker.type = visualization_msgs::Marker::CUBE;
            float speed = sqrt(object.velocity.linear.x * object.velocity.linear.x +
                               object.velocity.linear.y * object.velocity.linear.y) *
                          3.6;
            marker.text = std::to_string(object.id);
            marker.pose.position = object.pose.position;
            marker.pose.position.z += 0.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            if (object.label == "Traffic_light" || object.label == "Right_traffic_light")
            {
                std::string color_str = aipe::TLUtils::GetTLStatus(object);
                bool color_available = aipe::TLUtils::GetTLMarkerColor(color_str, marker.color.r, marker.color.g, marker.color.b);
                if (!color_available)
                {
                    //ROS_INFO("[%s] Color is not available!", __APP_NAME__);
                    //continue;
                }
            }
            marker.color.a = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 1;

            marker.lifetime = ros::Duration(1);
            final_markers.markers.push_back(marker);
            visualization_msgs::Marker text_marker = marker;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.text = std::to_string(text_marker.id);
            text_marker.scale.z = 2;
            //text_marker.pose.position.x += 1;
            if (text_marker.id % 2 == 0)
                text_marker.pose.position.y -= 0.5;
            else
                text_marker.pose.position.y += 0.5;
            final_markers.markers.push_back(text_marker);
            i++;
        }
    }
    return final_markers;
}

visualization_msgs::MarkerArray aipe::TrafficLightControlApp::ObjectsToMarkers(const aipe_msgs::DetectedObjectArray &in_objects, const std::string &final_state)
{
    visualization_msgs::MarkerArray final_markers;

    //if (in_objects.objects.size() > 3)
    if (!in_objects.objects.empty())
    {
        visualization_msgs::Marker marker;
        marker.header = in_objects.objects[0].header;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = 15;
        marker.pose.position.z = 2;
        //marker.pose.position.x = (in_objects.objects[0].pose.position.x + in_objects.objects[(in_objects.objects.size() / 2) - 1].pose.position.x) / 2;
        //marker.pose.position.y = (in_objects.objects[0].pose.position.y + in_objects.objects[(in_objects.objects.size() / 2) + 1].pose.position.y) / 2;
        //marker.pose.position.z = in_objects.objects[0].pose.position.z;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        //std::cout << "Final state: " << final_state << std::endl;
        bool color_available = aipe::TLUtils::GetTLMarkerColor(final_state, marker.color.r, marker.color.g, marker.color.b);
        if (!color_available)
        {
            //std::cout << "Final state: " << final_state << std::endl;
            //ROS_INFO("[%s] Color is not available!", __APP_NAME__);
            //continue;
        }

        marker.color.a = 1.0;
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;

        marker.lifetime = ros::Duration(1);
        final_markers.markers.push_back(marker);
    }

    return final_markers;
}

void aipe::TrafficLightControlApp::FilterClassifications(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications, aipe_msgs::ClassificationArray::Ptr &filtered_classifications)
{
    filtered_classifications->classifications.clear();
    filtered_classifications->header = in_classifications->header;
    for (auto obj : in_classifications->classifications)
    {
        if (std::find(CAS_color_list_.begin(), CAS_color_list_.end(), obj.class_name) != CAS_color_list_.end())
        {
            filtered_classifications->classifications.push_back(obj);
        }
    }
}

void aipe::TrafficLightControlApp::ClassificationsFTCCallback(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications)
{
    if (classifications_tbc_)
    {
        tbc_time_ = std::chrono::high_resolution_clock::now();
        FilterClassifications(in_classifications, classifications_tbc_);
    }
}

void aipe::TrafficLightControlApp::ClassificationsFBLCallback(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications)
{
    if (classifications_tbl_)
    {
        tbl_time_ = std::chrono::high_resolution_clock::now();
        FilterClassifications(in_classifications, classifications_tbl_);
    }
}

void aipe::TrafficLightControlApp::ClassificationsFBRCallback(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications)
{
    if (classifications_tbr_)
    {
        tbr_time_ = std::chrono::high_resolution_clock::now();
        FilterClassifications(in_classifications, classifications_tbr_);
        classifications_tbr_raw_->classifications.clear();
        classifications_tbr_raw_->header = in_classifications->header;
        for (auto obj : in_classifications->classifications)
        {
            classifications_tbr_raw_->classifications.push_back(obj);
        }
    }
}

void aipe::TrafficLightControlApp::PublishOvershooting(const bool &detection_available)
{
    // TODO: Off traffic lights are filtered out on sensor fusion. They must be received (externally) to make this flag on.
    std::string loc_id_str = target_location_id_;
    int img_width = -1;
    boost::algorithm::to_lower(loc_id_str);
    if (loc_id_str.find("l1") != std::string::npos)
    {
        // tbr
        img_width = right_cam_width_;
    }
    else if (loc_id_str.find("l1") != std::string::npos)
    {
        //tbl
        img_width = left_cam_width_;
    }
    std_msgs::Int64 overshoot_msg;
    overshoot_msg.data = 0;
    if (apm_status_ == 18 && !detection_available && CAS_tl_x_coordinate_ != -1)
    {
        if (CAS_tl_x_coordinate_ > 0.6 * img_width)
        {
            overshoot_msg.data = -1;
        }
        else
        {
            overshoot_msg.data = 1;
        }
    }
    publisher_overshoot_.publish(overshoot_msg);
}

void aipe::TrafficLightControlApp::PublishTrafficLightLog(const aipe_msgs::Classification &output)
{
    tl_log_.header = output.header;
    tl_log_.apm_status = std::to_string(apm_status_);
    //tl_log_.classifications = *classifications_tbc_;
    tl_log_.current_track_index = current_lane_;
    tl_log_.detections = detected_objs_;
    tl_log_.light_indicator = light_indicator_;
    tl_log_.tl_signal = output;
    publisher_tl_log_.publish(tl_log_);
}

void aipe::TrafficLightControlApp::BayAnalysis(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg)
{
    std::string final_state = "";
    int red_age = 0;
    int red_invisible = 0;
    bool detection_available = false;

    // estimate CAS TL state using relevant side camera
    bool state_valid = CAS_estimator_->JunctionState(in_msg, target_location_id_, classifications_tbc_, classifications_tbr_,
                                                     tbr_time_, classifications_tbl_, tbl_time_, apm_status_, final_state,
                                                     red_age, red_invisible, CAS_tl_x_coordinate_, detection_available);

    // Publish final state
    aipe_msgs::Classification CAS_state;
    CAS_state.header = in_msg->header;
    CAS_state.class_name = final_state;
    boost::algorithm::to_lower(CAS_state.class_name);
    CAS_state.confidence = 1;
    CAS_state.class_index = aipe::TLUtils::GetClassIndex(CAS_state.class_name);
    // TODO: Temporary added to prevent extra integration work
    CAS_state.x = red_age;
    publisher_junction_.publish(CAS_state);
    // publish inputs and output of the node
    PublishTrafficLightLog(CAS_state);
    PublishClassification(publisher_tl_cas_, in_msg->header, final_state, 1);

    // Publish precision alignment flag true when APM is close to the final destination
    std_msgs::Bool prec_align_msg;
    prec_align_msg.data = 0;
    // if the traffic light is flickering with red
    //std::cout << "Redinvisible: " << red_invisible << std::endl;
    if (red_invisible < precision_alignment_thresh_)
    {
        prec_align_msg.data = 1;
    }
    publisher_precision_alignment_.publish(prec_align_msg);

    //overshooting scenario
    //PublishOvershooting(detection_available);
}

void aipe::TrafficLightControlApp::JunctionAnalysis(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg, std::string &final_state, float &state_conf)
{
    aipe_msgs::DetectedObjectArray::Ptr tl_detections = boost::shared_ptr<aipe_msgs::DetectedObjectArray>(new aipe_msgs::DetectedObjectArray);
    for (auto obj : in_msg->objects)
    {
        if (obj.label == "Traffic_light" || obj.label == "Right_traffic_light")
        {
            tl_detections->objects.push_back(obj);
        }
    }

    bool state_valid = junction_estimator_->JunctionState(tl_detections, classifications_tbr_raw_, junction_id_, light_indicator_, current_lane_,
                                                          junction_hazard_, final_state, state_conf);
}

void aipe::TrafficLightControlApp::DetectionsCallback(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg)
{
    detected_objs_ = *in_msg;
    std::string final_state = "";
    float state_conf = 0.0f;
    if (inside_bay_)
    {
        BayAnalysis(in_msg);
    }
    else
    {
        JunctionAnalysis(in_msg, final_state, state_conf);
        PublishClassification(publisher_tl_junction_, in_msg->header, final_state, state_conf);
        PublishTLSignal(in_msg, final_state, state_conf);
    }

    // cas and junction state must be published on each iteration so publish the empty one (They can't be valid at the same time)
    if (inside_bay_)
    {
        PublishClassification(publisher_tl_junction_, in_msg->header, "", 0.0f);
    }
    else
    {
        PublishClassification(publisher_tl_cas_, in_msg->header, "", 0.0f);
    }
}

aipe_msgs::Classification aipe::TrafficLightControlApp::PublishClassification(const ros::Publisher publisher, const std_msgs::Header &header,
                                                                              const std::string &final_state, const float &state_conf)
{
    aipe_msgs::Classification junction_state;
    junction_state.header = header;
    junction_state.class_name = final_state;
    boost::algorithm::to_lower(junction_state.class_name);
    junction_state.confidence = state_conf;
    junction_state.class_index = aipe::TLUtils::GetClassIndex(junction_state.class_name);
    publisher.publish(junction_state);
    return junction_state;
}

bool aipe::TrafficLightControlApp::PublishTLSignal(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg,
                                                   const std::string &final_state, const float &state_conf)
{
    bool status = true;

    visualization_msgs::MarkerArray junction_state_marker = ObjectsToMarkers(*in_msg, final_state);
    publisher_junction_marker_.publish(junction_state_marker);

    aipe_msgs::Classification junction_state = PublishClassification(publisher_junction_, in_msg->header, final_state, state_conf);
    // publish inputs and output of the node
    PublishTrafficLightLog(junction_state);
    return status;
}

void aipe::TrafficLightControlApp::SpeedCallback(const airs_msgs::reg37::ConstPtr &in_apm_speed)
{
    // km/h to m/s
    std::string received_speed = in_apm_speed->Vehicle_Speed_Feedback;
    apm_speed_ = std::atof(received_speed.c_str()) * 0.2777777;
}

void aipe::TrafficLightControlApp::LaneCallback(const std_msgs::String::ConstPtr &in_lane)
{
    current_lane_ = in_lane->data;
}

void aipe::TrafficLightControlApp::LightIndicatorCallback(const std_msgs::Int64::ConstPtr &in_light_indicator)
{
    light_indicator_ = in_light_indicator->data;
}

void aipe::TrafficLightControlApp::HazardCallback(const aisc_msgs::SituationCognition::ConstPtr &in_hazard)
{
    if (in_hazard)
    {
        junction_hazard_ = in_hazard->tl_junction_hazard;
    }
    else
    {
        ROS_INFO("[%s] Received hazard messages is empty. Junction hazard situation set to true!", __APP_NAME__);
        junction_hazard_ = true;
    }
}

void aipe::TrafficLightControlApp::TripJobCallback(const aifo_client::TripJobStatus::ConstPtr &in_trip_job_status)
{
    if (in_trip_job_status)
    {
        target_location_id_ = in_trip_job_status->target_location_id;
    }
    else
    {
        ROS_INFO("[%s] Received trip job status is empty!", __APP_NAME__);
    }
}

void aipe::TrafficLightControlApp::ApmStatusCallback(const std_msgs::Int64::ConstPtr &in_apm_status)
{
    if (in_apm_status)
    {
        apm_status_ = in_apm_status->data;
        if (apm_status_ == 18)
        {
            inside_bay_ = true;
        }
        else
        {
            inside_bay_ = false;
        }
    }
    else
    {
        ROS_INFO("[%s] Received apm status is empty!", __APP_NAME__);
    }
}

void aipe::TrafficLightControlApp::CameraInfoCallback(const sensor_msgs::CameraInfo &in_message)
{
    if (in_message.width > 0)
    {
        top_cam_width_ = in_message.width;
        cam_info_subscriber_.shutdown();
    }
    ROS_INFO("[%s] camera_top_center_front camera intrinsics obtained.", __APP_NAME__);
}

void aipe::TrafficLightControlApp::JunctionIdCallback(const std_msgs::String::ConstPtr in_msg)
{
    if (in_msg)
    {
        junction_id_ = in_msg->data;
    }
}

void aipe::TrafficLightControlApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
    //get params
    float overlap_threshold = 0.5;
    std::string fused_objects_topic = "/aipe/fused/tracked_objects";
    std::string classifications_fbc_topic = "/aipe/camera_top_center_front/classifications";
    std::string classifications_fbl_topic = "/aipe/camera_top_left_front/classifications";
    std::string classifications_fbr_topic = "/aipe/camera_top_right_front/classifications";
    std::string tl_control_topic_str = "aipe/traffic_light/signal_tl";
    std::string tl_cas_topic = "aipe/tl_signal/cas";
    std::string tl_junction_topic = "aipe/tl_signal/junction";
    std::string tl_control_marker_topic = "aipe/traffic_light/signal_tl/marker";
    std::string traffic_lights_topic = "/aipe/traffic_lights";
    std::string tl_markers_str = "/aipe/traffic_lights/markers";
    std::string apm_speed_src_topic = "/airs/plc/critical_fbk/reg_37";
    std::string lane_src_topic = "/aidc/run_iterator/current_track_index";
    std::string light_indicator_src_topic = "/aidc/run_iterator/light/indicator";
    std::string hazard_src_topic = "/aisc/situation_cognition/result";
    std::string trip_job_topic = "/aifo/trip_job/status";
    std::string apm_status_topic = "/aidc/apm_status/status";
    std::string cam_info_topic = "/camera_top_center_front/camera_info";
    std::string precision_alignment_topic = "/aipe/precision_alignment_tl";
    std::string overshooting_topic = "/aipe/traffic_light/CAS_overshooting";
    std::string tl_log_topic = "/aipe/traffic_light/log";
    std::string junc_id_topic = "/aipe/junction_id";
    std::string lanes_file = "";
    bool virtual_poles = true;
    std::string workspace_dir = ros::package::getPath("psa_apm_core_config");

    ROS_INFO("[%s] This node requires: vision based traffic light detections being published.", __APP_NAME__);
    in_private_handle.param<std::string>("/tl_vision_control/fused_objects", fused_objects_topic, "/aipe/fused/tracked_objects");
    ROS_INFO("[%s] fused_objects: %s", __APP_NAME__, fused_objects_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/classifications_fbc_topic", classifications_fbc_topic, "/aipe/camera_top_center_front/classifications");
    ROS_INFO("[%s] fbc classifications: %s", __APP_NAME__, classifications_fbc_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/classifications_fbl_topic", classifications_fbl_topic, "/aipe/camera_top_left_front/classifications");
    ROS_INFO("[%s] fbl classifications: %s", __APP_NAME__, classifications_fbl_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/classifications_fbr_topic", classifications_fbr_topic, "/aipe/camera_top_right_front/classifications");
    ROS_INFO("[%s] fbr classifications: %s", __APP_NAME__, classifications_fbr_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/apm_speed_src", apm_speed_src_topic, "/airs/plc/critical_fbk/reg_37");
    ROS_INFO("[%s] apm_speed_src: %s", __APP_NAME__, apm_speed_src_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/lane_src_topic", lane_src_topic, "/aidc/run_iterator/current_track_index");
    ROS_INFO("[%s] lane_src_topic: %s", __APP_NAME__, lane_src_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/light_indicator_src_topic", light_indicator_src_topic, "/aidc/run_iterator/light/indicator");
    ROS_INFO("[%s] light_indicator_src_topic: %s", __APP_NAME__, light_indicator_src_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/hazard_topic", hazard_src_topic, "/aisc/situation_cognition/result");
    ROS_INFO("[%s] hazard_topic: %s", __APP_NAME__, hazard_src_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/trip_job_topic", trip_job_topic, "/aifo/trip_job/status");
    ROS_INFO("[%s] trip_job_topic: %s", __APP_NAME__, trip_job_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/apm_status_topic", apm_status_topic, "/aidc/apm_status/status");
    ROS_INFO("[%s] apm_status_topic: %s", __APP_NAME__, apm_status_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/tl_cas_topic", tl_cas_topic, "/aipe/tl_signal/cas");
    ROS_INFO("[%s] tl_cas_topic: %s", __APP_NAME__, tl_cas_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/tl_junction_topic", tl_junction_topic, "/aipe/tl_signal/junction");
    ROS_INFO("[%s] tl_junction_topic: %s", __APP_NAME__, tl_junction_topic.c_str());

    in_private_handle.param<std::string>("/v2i_control/junction_id_topic", junc_id_topic, "/aipe/junction_id");
    ROS_INFO("[%s] junction_id_topic: %s", __APP_NAME__, junc_id_topic.c_str());

    in_private_handle.param<std::string>("/tl_vision_control/lanes_file", lanes_file, "src/aipe/src/tl_vision_control/src/cfg/lanes.json");
    //lanes_file = workspace_dir + '/' + lanes_file;
    ROS_INFO("[%s] lanes_file: %s", __APP_NAME__, lanes_file.c_str());

    in_private_handle.param<float>("/tl_vision_control/overlap_threshold", overlap_threshold, 0.01);
    ROS_INFO("[%s] overlap_threshold: %f", __APP_NAME__, overlap_threshold);

    in_private_handle.param<bool>("/tl_vision_control/virtual_poles", virtual_poles, true);
    ROS_INFO("[%s] virtual_poles: %f", __APP_NAME__, virtual_poles);

    int flickering_duration = 3; //in frames
    in_private_handle.param<int>("/tl_vision_control/flickering_duration", flickering_duration, 3);
    ROS_INFO("[%s] flickering_duration: %d", __APP_NAME__, flickering_duration);

    int red_tolerance = 3; //in frames
    in_private_handle.param<int>("/tl_vision_control/red_tolerance", red_tolerance, 3);
    ROS_INFO("[%s] red_tolerance: %d", __APP_NAME__, red_tolerance);

    in_private_handle.param<int>("/tl_vision_control/max_CAS_duration", max_CAS_duration_, 50);
    ROS_INFO("[%s] max_CAS_duration: %d", __APP_NAME__, max_CAS_duration_);

    int turn_right_wait_time = 0;
    in_private_handle.param<int>("/tl_vision_control/turn_right_wait_time", turn_right_wait_time, 20);
    ROS_INFO("[%s] turn_right_wait_time: %d", __APP_NAME__, turn_right_wait_time);

    in_private_handle.param<int>("/tl_vision_control/precision_alignment_thresh", precision_alignment_thresh_, 1);
    ROS_INFO("[%s] precision_alignment_thresh: %d", __APP_NAME__, precision_alignment_thresh_);

    float max_v2i_dist = 50;
    in_private_handle.param<float>("/tl_vision_control/v2i_max_dist_to_junction", max_v2i_dist, 50);
    ROS_INFO("[%s] v2i_max_dist_to_junction: %f", __APP_NAME__, max_v2i_dist);

    float v2i_time_buffer = 2000;
    in_private_handle.param<float>("/tl_vision_control/v2i_time_buffer", v2i_time_buffer, 2000);
    ROS_INFO("[%s] v2i_time_buffer: %f", __APP_NAME__, v2i_time_buffer);

    //generate subscribers and sychronizers
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, fused_objects_topic.c_str());
    detections_tl_subscriber_ = in_private_handle.subscribe(fused_objects_topic,
                                                            1,
                                                            &aipe::TrafficLightControlApp::DetectionsCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, classifications_fbc_topic.c_str());
    classifications_fbc_subscriber_ = in_private_handle.subscribe(classifications_fbc_topic,
                                                                  1,
                                                                  &aipe::TrafficLightControlApp::ClassificationsFTCCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, classifications_fbl_topic.c_str());
    classifications_fbl_subscriber_ = in_private_handle.subscribe(classifications_fbl_topic,
                                                                  1,
                                                                  &aipe::TrafficLightControlApp::ClassificationsFBLCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, classifications_fbr_topic.c_str());
    classifications_fbr_subscriber_ = in_private_handle.subscribe(classifications_fbr_topic,
                                                                  1,
                                                                  &aipe::TrafficLightControlApp::ClassificationsFBRCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, apm_speed_src_topic.c_str());
    speed_subscriber_ = in_private_handle.subscribe(apm_speed_src_topic,
                                                    1,
                                                    &aipe::TrafficLightControlApp::SpeedCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, lane_src_topic.c_str());
    lane_subscriber_ = in_private_handle.subscribe(lane_src_topic,
                                                   1,
                                                   &aipe::TrafficLightControlApp::LaneCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, light_indicator_src_topic.c_str());
    light_indicator_subscriber_ = in_private_handle.subscribe(light_indicator_src_topic,
                                                              1,
                                                              &aipe::TrafficLightControlApp::LightIndicatorCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, hazard_src_topic.c_str());
    hazard_subscriber_ = in_private_handle.subscribe(hazard_src_topic,
                                                     1,
                                                     &aipe::TrafficLightControlApp::HazardCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, trip_job_topic.c_str());
    trip_job_subscriber_ = in_private_handle.subscribe(trip_job_topic,
                                                       1,
                                                       &aipe::TrafficLightControlApp::TripJobCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, apm_status_topic.c_str());
    apm_status_subscriber_ = in_private_handle.subscribe(apm_status_topic,
                                                         1,
                                                         &aipe::TrafficLightControlApp::ApmStatusCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, cam_info_topic.c_str());
    cam_info_subscriber_ = in_private_handle.subscribe(cam_info_topic,
                                                       1,
                                                       &aipe::TrafficLightControlApp::CameraInfoCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, junc_id_topic.c_str());
    junction_id_subscriber_ = in_private_handle.subscribe(junc_id_topic,
                                                          1,
                                                          &aipe::TrafficLightControlApp::JunctionIdCallback, this);

    publisher_junction_ = node_handle_.advertise<aipe_msgs::Classification>(tl_control_topic_str, 1);

    publisher_junction_marker_ = node_handle_.advertise<visualization_msgs::MarkerArray>(tl_control_marker_topic, 1);
    publisher_traffic_lights_ = node_handle_.advertise<aipe_msgs::DetectedObjectArray>(traffic_lights_topic, 1);
    publisher_tl_markers_ = node_handle_.advertise<visualization_msgs::MarkerArray>(tl_markers_str, 1);
    publisher_precision_alignment_ = node_handle_.advertise<std_msgs::Bool>(precision_alignment_topic, 1);
    publisher_overshoot_ = node_handle_.advertise<std_msgs::Int64>(overshooting_topic, 1);
    publisher_tl_log_ = node_handle_.advertise<aipe_msgs::TrafficLightLog>(tl_log_topic, 1);
    publisher_tl_cas_ = node_handle_.advertise<aipe_msgs::Classification>(tl_cas_topic, 1);
    publisher_tl_junction_ = node_handle_.advertise<aipe_msgs::Classification>(tl_junction_topic, 1);

    ROS_INFO("[%s] Publishing traffic light control in %s", __APP_NAME__, tl_control_topic_str.c_str());
    ROS_INFO("[%s] Publishing cas tl signal in %s", __APP_NAME__, tl_cas_topic.c_str());
    ROS_INFO("[%s] Publishing junction tl signal in %s", __APP_NAME__, tl_junction_topic.c_str());
    ROS_INFO("[%s] Publishing traffic lights in %s", __APP_NAME__, traffic_lights_topic.c_str());
    ROS_INFO("[%s] Publishing traffic light junction marker in %s", __APP_NAME__, tl_control_marker_topic.c_str());
    ROS_INFO("[%s] Publishing traffic light markers in %s", __APP_NAME__, tl_markers_str.c_str());
    ROS_INFO("[%s] Publishing precision alignment flag in %s", __APP_NAME__, precision_alignment_topic.c_str());
    ROS_INFO("[%s] Publishing traffic light log in %s", __APP_NAME__, tl_log_topic.c_str());

    junction_estimator_ = boost::shared_ptr<aipe::JunctionStateEstimator>(new aipe::JunctionStateEstimator(lanes_file, virtual_poles, turn_right_wait_time, top_cam_width_));
    CAS_estimator_ = boost::shared_ptr<aipe::CASStateEstimator>(new aipe::CASStateEstimator(flickering_duration, red_tolerance, CAS_color_list_, max_CAS_duration_));
}

void aipe::TrafficLightControlApp::Run()
{
    ros::NodeHandle private_node_handle("~");

    InitializeRosIo(private_node_handle);

    ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

    ros::spin();

    ROS_INFO("[%s] END", __APP_NAME__);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, __APP_NAME__);

    aipe::TrafficLightControlApp app;

    app.Run();

    return 0;
}
