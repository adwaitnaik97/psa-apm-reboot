#ifndef _AVCS_HANDLER_HPP_
#define _AVCS_HANDLER_HPP_


#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <aios_apm_msgs/AiosPreprocess.h>
#include <aios_apm_msgs/AiosSessionInfo.h>
#include <aios_apm_msgs/AiosSessionPreprocess.h>
#include <aios_apm_msgs/HttpsClient.h>
#include <aios_apm_msgs/GuiControl.h>
#include <aios_apm_msgs/AifoStatus.h>
#include <aios_apm_msgs/ContainerDB.h>
#include <aios_apm_msgs/HttpsClientHeader.h>
#include <aios_apm_msgs/GetSlotIDCoordinates.h>
#include <aios_apm_msgs/JobInfo.h>
#include <aios_apm_msgs/JobPath.h>
#include <aios_apm_msgs/V2eInfo.h>

#include <aide_apm_msgs/ZoneInfo.h>
#include <aide_apm_msgs/GetZoneInfo.h>
#include "aios_apm_msgs/GetLaneInfo.h"
#include "aios_apm_msgs/OntologyPath.h"
#include "aios_apm_msgs/OntologyWaypoint.h"


// boost headers
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/lexical_cast.hpp>  // see reference at end of file
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.


#include <XmlRpcValue.h>
#include <string> 
#include <time.h>       /* time_t, struct tm, time, localtime, strftime */

#include <locale>
#include <iostream>
#include <fstream>

#include <nlohmann/json.hpp>

#include <gps_common/conversions.h>

// aidc_msg Headers
#include <aidc_msgs/Trajectory.h>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>




class AifoHandler {
	public:
		//public functions
  		AifoHandler(ros::NodeHandle nh, ros::NodeHandle private_nh);
		~AifoHandler();

	void publishAifoStatus();
	void checkLogoffRequest();
	void checkTrailerUpdates(); 

	void establishConnectivityToFMS(); 
    void alignDockRequest(); 
  	void timerCallback(const ros::TimerEvent &event);
  	void timerCallback2(const ros::TimerEvent &event);
  	void timerCallback3(const ros::TimerEvent &event);

    void aiosPreprocessCallback(const aios_apm_msgs::AiosPreprocess::ConstPtr& msg); 
    void aiosInstructionsCallback(const std_msgs::String::ConstPtr &msg);
    void aiosAcceptedPathCallback(const aios_apm_msgs::JobPath::ConstPtr& msg); 

    void refuelCallback(const std_msgs::Int64::ConstPtr& msg); 
    void rvizSelectedPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); 

  	void checkJobStatusUpdates();
  	void updateAVCSJobStatus();
    void handle_avcs_job_request(nlohmann::json aifo_response_msg);

    void handle_avcs_logon_request(nlohmann::json aifo_response_msg);

    void handle_job_cancel_request(nlohmann::json aifo_response_msg);

    void handle_stop_job_request(nlohmann::json aifo_response_msg);

    void handle_resume_job_request(nlohmann::json aifo_response_msg);

    void handle_manual_route_request(nlohmann::json aifo_response_msg);

    void handle_manual_route_confirmation_request(nlohmann::json aifo_response_msg);

    void handle_armg_instruction_to_v2e(nlohmann::json aifo_response_msg);

    void handle_path_update_request(nlohmann::json aifo_response_msg);
    void handle_link_up_request(nlohmann::json aifo_response_msg);

    void handle_lane_speed_limit_request(nlohmann::json aifo_response_msg);

    void handle_lane_block_request(nlohmann::json aifo_response_msg);

    void handle_avcs_map(nlohmann::json aifo_response_msg);
    void handle_block_and_speedcontrol_request(nlohmann::json aifo_response_msg);

    void aios_complete_jobCallback(const std_msgs::Bool::ConstPtr& msg); 

    void aifoResponseBLOCKCallback(const std_msgs::String::ConstPtr& msg);

    void aidePoseQueryFeedbackCallback(const aide_apm_msgs::ZoneInfo::ConstPtr& msg); 

    void aifoResponseCallback(const std_msgs::String::ConstPtr& msg); 

    void actionReqCallback(const aios_apm_msgs::GuiControl::ConstPtr& msg); 
    void reset_flags_power();
    void resetJobProgressOnNewInstructions();

    void reset_flags_logon();

    auto construct_fault_message();

    auto construct_lc_message();

    auto construct_root_message(nlohmann::json aios_req, nlohmann::json aios_res);

    auto publish_payload(nlohmann::json root_msg);

   bool power_on_request();

    bool update_trailer_request();

    bool power_off_request();

    bool transition_request(int mode);

    void logout_request();

    auto job_request();

    void job_dest_reached();

    void handle_avcs_op_request(nlohmann::json aifo_response_msg);
    void checkExceptionHandle();

    auto format_job_id(auto destination_label, int job_type);

    void handle_mount_request(nlohmann::json aifo_response_msg);

    void handle_offload_request(nlohmann::json aifo_response_msg);

    void handle_dock_request(nlohmann::json aifo_response_msg);

    void handle_avcs_maintenance_request(nlohmann::json aifo_response_msg);

    void handle_avcs_parking_request(nlohmann::json aifo_response_msg);

    void handle_avcs_refuel_request(nlohmann::json aifo_response_msg);
    void initialiseMqttComms();
	nlohmann::json parseJsonFile(std::string param_name, ros::NodeHandle nh);

  	ros::NodeHandle nh2;

    ros::Timer timer;
    ros::Timer timer2;
	void checkLastKnownOperationStatus();
	void publishAVCSResponses();
	void retrieveLastMap();
	void checkLastKnownJobStatus();
	void sendAVCS_HB_and_res();
	nlohmann::json saveToCsvFile();
	std::string retrieve_lane_id(geometry_msgs::Pose2D query_point);
    std::string retrieve_lane_id(nlohmann::json query_json);
    bool check_validity_condition(nlohmann::json json_aios_res_xx_response, bool reject_condition_met, std::string reject_msg);


	int mi_count = 0;
    aios_apm_msgs::JobInfo retriveJobInfoFromInstruction(nlohmann::json avcs_instruction_msg, int instruction_type, bool precheck=false);
   
    void handleNavigationRequestInstructions(nlohmann::json aifo_response_msg, int requested_instruction_type);
	public:
		//public variables
	    bool apm_preprocess_initialized = false;
	    bool fms_connected = false;
	    std::string apm_id = "";






  	private:
    	// Declare publishers, subscribers and services
		ros::Subscriber aios_preprocess_sub_;
		ros::Subscriber aios_instructions_sub_;
		ros::Subscriber aios_converted_path_sub_;
	    ros::Subscriber avcs_response_sub_;
	    ros::Subscriber avcs_map_response_sub_;

	    ros::Subscriber action_request_sub_;
	    ros::Subscriber refuel_sub;
	    ros::Subscriber pose_query_feedback_sub;
	    ros::Subscriber rviz_map_point_query_sub;

	    ros::ServiceClient job_info_srv;
	    ros::ServiceClient zone_info_srv;
	    ros::ServiceClient ontologyinfo_srv;

	    ros::Publisher session_status_pub_;
	    ros::Publisher avcs_requests_pub_;
	    ros::Publisher new_job_pub_;
	    ros::Publisher new_job_coord_pub_;
	    ros::Publisher job_complete_pub_;
	    ros::Publisher job_id_pub_;
	    ros::Publisher aios_diagnostics_pub_;
	    ros::Publisher blocked_lanes_pub_;
	    ros::Publisher lanes_with_reduced_speed_pub_;
	    ros::Publisher global_path_res_pub_;
	    ros::Publisher pose_query_pub_;
	    ros::Publisher manual_route_lane_id_pub_;
	    ros::Publisher manual_route_trajectory_pub_;
	    ros::Publisher armg_instruction_pub_;
	    ros::Publisher v2e_info_pub_;
	    ros::Publisher v2e_dist_pub_;
	    ros::Publisher trajectory_pcl_pub_;
	    ros::Publisher given_route_pcl_pub_;
	    ros::Publisher avcs_map_points_pub_;
	    ros::Publisher debug_avcs_map_point_name_pub_;
	    ros::Publisher debug_avcs_opmode_pub_;
	    ros::Publisher debug_avcs_response_pub_;

	    ros::Publisher debug_fr_container_pub_;
	    ros::Publisher debug_bk_container_pub_;
	    ros::Publisher debug_fr_container_id_pub_;
	    ros::Publisher debug_bk_container_id_pub_;

	    ros::Publisher road_block_xya_pub_;
	    ros::Publisher speed_restrict_xya_pub_;

        std_msgs::Float32 fr_contr_pos;
	    std_msgs::Float32 bk_contr_pos;
	    std_msgs::String fr_contr_id;
	    std_msgs::String bk_contr_id;

        std_msgs::String closest_point_name;

        std_msgs::String viz_avcs_response_msg;
      	aios_apm_msgs::V2eInfo v2e_info_msg; 


    


	    std::string mqtt_subtopic_instructions;
	    std::string mqtt_subtopic_maps;
	    std::string mqtt_subtopic_maps2;
	    std::string mqtt_subtopic_blocks;
	    std::string mqtt_pubtopic_general;

	    std::string aios_payload_intopic_instructions;
	    std::string aios_payload_inttopic_maps;
	    std::string aios_payload_outttopic;

	    std::string intopic_aios_preprocess_;
	    std::string intopic_aios_instructions_;
	    std::string output_topic_session_status_;

	    std::string output_status_msg_csv_file;
	    std::string output_map_csv_file;
	    std::string output_converted_map_csv_file;
	    std::string output_reformat_map_csv_file;


	    aios_apm_msgs::AiosPreprocessConstPtr apm_preprocess_msg_ptr; 
	    
	    aios_apm_msgs::AiosPreprocess apm_preprocess_msg; 
	    aios_apm_msgs::AifoStatus avcs_status_msg; 

	    nlohmann::json json_aios_req_apm_move;
	    nlohmann::json json_aios_req_dest_arrived;
	    nlohmann::json json_aios_req_logoff;
	    nlohmann::json json_aios_req_new_job;
	    nlohmann::json json_aios_req_poweroff;
	    nlohmann::json json_aios_req_poweron;
	    nlohmann::json json_aios_req_update_trailer;
	    nlohmann::json json_aios_req_mode_change_update;
	    nlohmann::json json_aios_res_switch_mode;
	    nlohmann::json json_avcs_res_logon;
	    nlohmann::json json_hb_field_fault_req;
	    nlohmann::json json_hb_field_pose_req;
	    nlohmann::json json_hb_root_apm_to_avcs;
	    nlohmann::json json_hb_root_avcs_to_apm;
	    nlohmann::json json_aios_res_job_response;
	    nlohmann::json json_avcs_req_job_instruction;
	    nlohmann::json json_avcs_req_dock;
	    nlohmann::json json_aios_res_dock_res;

	    nlohmann::json json_aios_res_mount_res;
	    nlohmann::json json_aios_res_offload_res;
	    nlohmann::json json_avcs_req_mount;
	    nlohmann::json json_avcs_req_offload;


	    nlohmann::json json_aios_res_refuel_response;
	    nlohmann::json json_avcs_req_refuel_instruction;
	    nlohmann::json json_aios_res_maintenance_response;
	    nlohmann::json json_avcs_req_maintenance_instruction;
	    nlohmann::json json_aios_res_parking_response;
	    nlohmann::json json_avcs_req_parking_instruction;
	    nlohmann::json json_aios_res_park_response;
	    nlohmann::json json_avcs_req_park_instruction;
	    nlohmann::json json_avcs_req_cancel_instruction;
	    nlohmann::json json_aios_res_cancel_res;
	    nlohmann::json json_aios_res_path_update;
	    nlohmann::json json_avcs_req_path_update;
	    nlohmann::json json_avcs_req_mi_req;
	    nlohmann::json json_avcs_req_linkup_req;
	    nlohmann::json json_aios_res_linkup_res;

	    nlohmann::json json_avcs_req_stop_job_instruction;
	    nlohmann::json json_aios_res_stop_job_res;
	    nlohmann::json json_avcs_req_resume_job_instruction;
	    nlohmann::json json_aios_res_resume_job_res;
	    nlohmann::json json_avcs_req_manual_route_instruction;
	    nlohmann::json json_aios_res_manual_route_response;
	    nlohmann::json json_avcs_req_manual_route_confirmation_request;
	    nlohmann::json json_aios_res_manual_route_confirmation_response;

	    nlohmann::json json_empty_json;
	    nlohmann::json json_aios_mqtt_config;
	    nlohmann::json json_avcs_req_next_job_instruction;
	    nlohmann::json json_avcs_road_blocked_next_job_instruction;

	    nlohmann::json json_avcs_converted_map_data;
	    nlohmann::json json_avcs_reformatted_map_data;
        nlohmann::json json_map_restrictions_data;
        nlohmann::json json_accepted_path_ll;
        nlohmann::json json_prev_accepted_path_ll;
        nlohmann::json json_response_history;


    	XmlRpc::XmlRpcValue frame_map_enum_list;

	    bool initial_power_off = false;
	    bool comms_established = false;
	    bool manual_route_responded = false;
	    bool second_call = false;
	    bool param_trim_road_blocks = true;
	    std::string param_version;
	    int fuel_level_avcs = 80;
		bool exception_responded = false;
		int exception_id_being_handled = 0;

	    int job_type;
	    bool dest_reached = false;
	    bool log_off_from_TN_required = false;
	    bool log_off_from_MA_required = false;
	    bool refuel_standby_complete = false;
	    bool refuel_handle_kiosk = false;
	    std_msgs::Bool set_job_complete;
	    std_msgs::Float32 aios_diagnostics_;
	    bool complete_job_for_new = false;
	    bool cancel_job_requested = false;
	    bool job_complete_trigered = false;
	    bool pause_job = false;
	    bool manual_route_job = false;
	    bool manual_route_job_accepted = false;
	    bool apm_doing_yard_job = false;
	    bool param_container_overide;
	    bool is_current_lane_id_pub = false;
	    std::string lane_to_block_for_manual_route = "";
	    float aios_diagnostics_value=4.0;
	    float expiry_duation_of_last_known_message;
	    float param_acceptable_distance_from_current;
	    float param_acceptable_squared_distance_from_current;
	    int instruction_job_type = 0;
	    int manual_route_instruction_job_type = 0;
	    int next_instruction_job_type = 0;
	    int next_road_block_instruction_job_type = 0;
	    int local_job_id = 0;
	    int fms_job_status = 0;
	    int force_override_job_count = 0;
	    std::string force_override_job_id = "";
	    std::string force_override_local_job_id = "";
    	bool accept_job_via_instructions_;

        int local_container_config = 0;
	    int v2e_msg_count = 0;
	    bool not_complete = false;
	    bool timer_active = false;
	    bool request_job_trigger = false;
	    bool dock_move_requested = false;
	    std_msgs::String blocked_lane_ids_to_publish;
	    std_msgs::String lanes_to_reduce_speed_publish;
	    std::vector<std::string> lanes_to_block;
	    std::vector<std::string> lanes_with_reduced_speed;
	    std::vector<std::string> manual_route_lane_ids;
	    aidc_msgs::Trajectory manual_path_trajectory;
	    aios_apm_msgs::OntologyPath road_blocks_xya;
	    aios_apm_msgs::OntologyPath speed_restrict_xya;

	    std::vector<std::string> segments_to_reduce_speed;
	    std::vector<std::string> blocked_segment_names;
	    std::vector<std::string> manual_route_path_;

	    std::string current_lane_id;
	    nav_msgs::Odometry odom_msg_;

	    bool block_lane_id_fetch = false;
	    bool reduced_speed_lane_id_fetch = false;
	    bool param_viz = true;
	    bool path_received_from_session = false;
    	nlohmann::json json_pose_current_route_ll;
		nlohmann::json JSON_AVCS_JOB_STATUS;

	    std::string speed_limit_;
	    std::string client_id_prefix;

	    bool updated_lanes_to_block_request = false;
	    bool updated_lanes_with_reduced_speed_request = false;
	    bool manual_route_path_to_calculate = false;

	    std_msgs::String manual_route_str_;
	    std::string manual_route_str;
	    ros::Time last_hb_time;
	    ros::Time last_path_accepted_time;
	    ros::Time fms_connected_time;

    	std::ofstream StatusMsgCsvFile;
    	std::ofstream MapCsvFile;
    	std::ofstream ConvertedMapCsvFile;
    	std::ofstream ReformatedMapCsvFile;

    	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
        bool map_initialized = false;

		std::string broker_address;


};

void ROS_INFO_STREAM_COLOUR(const std::string& colour, auto text) {
    std::unordered_map<std::string, std::string> colourMap {
        {"black", "0;30"},
        {"red", "0;31"},
        {"green", "0;32"},
        {"yellow", "0;33"},
        {"blue", "0;34"},
        {"magenta", "0;35"},
        {"cyan", "0;36"},
        {"white", "0;37"}
    };

    auto it = colourMap.find(colour);
    if (it != colourMap.end()) {
        ROS_INFO_STREAM("\033[" << it->second << "m" << text << "\033[0m");
    } else {
        ROS_INFO_STREAM(text);
    }
}


 std::string formatDateTime(ros::Time ros_time, float offset=0.0, const char* format= "%Y-%m-%d %H:%M:%S") {
    time_t now = (ros_time + ros::Duration(offset * 3600)).toSec();
    std::tm* now_tm = localtime(&now);
    char buffer[20];
    strftime(buffer, sizeof(buffer), format, now_tm);
    return std::string(buffer);
}



// enum AiosJobStatus_AJS
// {
//   AJS_FREE = 0,
//   AJS_FREE_FOR_NEW_JOB = -1,
//   AJS_JOB_ACKNOWLEDGED = 1,
//   AJS_CAS_JOB_ACKNOWLEDGED = 2, // CAS - reset job info 
//   AJS_JOB_PLANNING = 3,
//   AJS_CAS_JOB_PLANNING = 4,     // CAS - Check type of cas required: dock pos changed? / fine alignment (V2E) etc
//   AJS_JOB_ACCEPTED = 5,
//   AJS_CAS_JOB_ACCEPTED = 6,     // CAS - Get AIDC ready to align (aidc apm_status = 18)
//   AJS_INPROGRESS = 7,
//   AJS_CAS_INPROGRESS = 8,       // CAS - perform alignment depending on type
//   AJS_REACHING_DST = 9,
//   AJS_CAS_ALIGN_STOPPED = 10,   // CAS - Not used
//   AJS_DST_REACHED = 11,
//   AJS_JOB_COMPLETE = 13,
//   AJS_CAS_DST_REACHED = 12,     // CAS - CAS complete (aidc apm_status = 19)
//   AJS_JOB_REJECTED = 21,
//   AJS_CAS_JOB_REJECTED = 22,
//   AJS_JOB_HALT_FOR_NEW_JOB = 30,
//   AJS_JOB_HALT_FOR_NEW_CAS_JOB = 31,
//   AJS_JOB_HALT_FOR_LOOP_JOB = 32,
//   AJS_JOB_FEATURE_ALIGN_QCRANE = 51,
//   AJS_JOB_FEATURE_RTG_QCRANE = 52,
//   AJS_JOB_PATH_VERIFY_ACKNOWLEDGE = 90,
//   AJS_JOB_PATH_VERIFY_PLANNING = 91,
//   AJS_JOB_PATH_VERIFY_ACCEPT = 92,
//   AJS_JOB_PATH_VERIFY_TIMEOUT = 93
// };







#endif