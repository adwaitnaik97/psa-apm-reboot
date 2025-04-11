/*******************************************************
 *
 *    _    ___  ___   ___  ___ __   __ ___  ___  ___
 *   /_\  |_ _||   \ | _ \|_ _|\ \ / /| __|| _ \/ __|
 *  / _ \  | | | |) ||   / | |  \ V / | _| |   /\__ \
 * /_/ \_\|___||___/ |_|_\|___|  \_/  |___||_|_\|___/
 *
 *
 * Copyright (C) 2022 AIOS @ AIDRIVERS Ltd - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * author = 'Mahi Nadesh'
 * email  = 'mahindan@aidrivers.ai'
 *
 *******************************************************/

// ROS standard msg Headers

#include <avcs_handler/avcs_handler.hpp>


AifoHandler::AifoHandler(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    ROS_INFO_STREAM("Retrieving parameters");

    nh.getParam("/avcs_handler/frame_map", frame_map_enum_list);


    private_nh.param<std::string>("intopic_aios_preprocess_", intopic_aios_preprocess_,"/aios/preprocess/info");
    private_nh.param<std::string>("output_topic_avcs_info", output_topic_session_status_,"info");
    private_nh.param<float>("expiry_duation_in_hrs_of_last_known_message", expiry_duation_of_last_known_message,2);
    private_nh.param<float>("acceptable_distance_from_current", param_acceptable_distance_from_current,6);
    param_acceptable_squared_distance_from_current = pow(param_acceptable_distance_from_current,2);
    private_nh.param<std::string>("status_msg_csv_filepath", output_status_msg_csv_file,"");
    private_nh.param<std::string>("map_csv_filepath", output_map_csv_file,"");
    private_nh.param<std::string>("converted_map_csv_filepath", output_converted_map_csv_file,"");
    private_nh.param<std::string>("reformat_map_csv_filepath", output_reformat_map_csv_file,"");
    private_nh.param<std::string>("/aios/avcs_handler/mqtt_client_node/broker_addr", avcs_status_msg.fms_status.fms_endpoint,"");
    private_nh.param<bool>("/avcs_handler/config/trim_road_blocks",param_trim_road_blocks,true);
    private_nh.param<std::string>("/avcs_handler/config/version",param_version,"v2.0.0");
    private_nh.param<bool>("/avcs_handler/config/container_overide",param_container_overide,true);
    private_nh.param<bool>("/avcs_handler/config/accept_job_via_instructions", accept_job_via_instructions_,false);
    nh.param<std::string>("intopic_aios_instructions", intopic_aios_instructions_,"/aios/instructions");

    //MQTT config_params
    json_aios_mqtt_config = AifoHandler::parseJsonFile("aios_mqtt_config",private_nh);
    int comm_type;
    nh.param<int>("/avcs_handler/mqtt/comm_type", comm_type,0);
    if(comm_type == 0)
      nh.param<std::string>("/avcs_handler/mqtt/local_broker_addr", broker_address,"");
    else if(comm_type == 1)
      nh.param<std::string>("/avcs_handler/mqtt/sit_broker_addr", broker_address,"");
    else if(comm_type == 2)
      nh.param<std::string>("/avcs_handler/mqtt/production_broker_addr", broker_address,"");
    else if(comm_type == -1)
    {
      avcs_status_msg.fms_status.connectivity.fms_connected = true;
      fms_connected = true;
    }


    nh.param<std::string>("/avcs_handler/mqtt/client_id_prefix", client_id_prefix,"");


    nh.param<std::string>("/avcs_handler/mqtt/mqtt_subtopic_instructions", mqtt_subtopic_instructions,"");
    nh.param<std::string>("/avcs_handler/mqtt/mqtt_subtopic_maps", mqtt_subtopic_maps,"");
    nh.param<std::string>("/avcs_handler/mqtt/mqtt_subtopic_maps2", mqtt_subtopic_maps2,"");
    nh.param<std::string>("/avcs_handler/mqtt/mqtt_subtopic_blocks", mqtt_subtopic_blocks,"");
    nh.param<std::string>("/avcs_handler/mqtt/mqtt_pubtopic_general", mqtt_pubtopic_general,"");

    nh.param<std::string>("/avcs_handler/mqtt/aios_payload_intopic_instructions", aios_payload_intopic_instructions,"");
    nh.param<std::string>("/avcs_handler/mqtt/aios_payload_inttopic_maps", aios_payload_inttopic_maps,"");
    nh.param<std::string>("/avcs_handler/mqtt/aios_payload_outttopic", aios_payload_outttopic,"");

    expiry_duation_of_last_known_message = expiry_duation_of_last_known_message * 60 * 60;
    aios_preprocess_sub_ = nh.subscribe(intopic_aios_preprocess_, 20,
                              &AifoHandler::aiosPreprocessCallback, this,
                              ros::TransportHints().tcpNoDelay(true));

    aios_converted_path_sub_ = nh.subscribe("/aios/session_management/info/path_ll", 1,
                              &AifoHandler::aiosAcceptedPathCallback, this,
                              ros::TransportHints().tcpNoDelay(true));
    avcs_response_sub_ = nh.subscribe("mqtt/from_client", 20,
                              &AifoHandler::aifoResponseCallback, this,
                              ros::TransportHints().tcpNoDelay(true));
    avcs_map_response_sub_ = nh.subscribe("mqtt/from_client/block", 20,
                              &AifoHandler::aifoResponseBLOCKCallback, this,
                              ros::TransportHints().tcpNoDelay(true));


    action_request_sub_ = nh.subscribe("config/action_req", 1,
                              &AifoHandler::actionReqCallback, this,
                              ros::TransportHints().tcpNoDelay(true));
    refuel_sub = nh.subscribe("/avcs/refuel", 5,
                              &AifoHandler::refuelCallback, this,
                              ros::TransportHints().tcpNoDelay(true));

    rviz_map_point_query_sub = nh.subscribe<sensor_msgs::PointCloud2>("/rviz_live_selected_points", 1, 
                              &AifoHandler::rvizSelectedPointsCallback, this,
                              ros::TransportHints().tcpNoDelay(true));




    aios_instructions_sub_ =
        nh.subscribe(intopic_aios_instructions_, 1,
                     &AifoHandler::aiosInstructionsCallback, this,
                     ros::TransportHints().tcpNoDelay(true));




    // pose_query_feedback_sub = nh.subscribe("/aide_query/pose_zones_feedback", 10,
    //                           &AifoHandler::aidePoseQueryFeedbackCallback, this,
    //                           ros::TransportHints().tcpNoDelay(true));


    avcs_requests_pub_ = nh.advertise<std_msgs::String>("mqtt/to_client", 1);
    blocked_lanes_pub_ = nh.advertise<std_msgs::String>("/aifo/road_block_track_id_string_avcs2", 10);
    lanes_with_reduced_speed_pub_ = nh.advertise<std_msgs::String>("/aifo/speed_reduction_track_id_string", 10);



    manual_route_trajectory_pub_ = nh.advertise<aidc_msgs::Trajectory>("/avcs/manual_route_path_trajectory2", 10);
    armg_instruction_pub_ = nh.advertise<std_msgs::String>("/v2e_client/payload_str", 10);
    v2e_info_pub_ = nh.advertise<aios_apm_msgs::V2eInfo>("/v2e_client/info", 1);
    v2e_dist_pub_ = nh.advertise<std_msgs::Float32>("/v2x/pose_offset", 1);


    trajectory_pcl_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
        "/viz/aios/avcs_handler/cloud/avcs_responded_path", 1, true);

    given_route_pcl_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
        "/viz/aios/avcs_handler/cloud/avcs_given_path", 1, true);

    avcs_map_points_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
        "/viz/aios/avcs_handler/cloud/avcs_map_points", 1, true);


    job_info_srv = nh.serviceClient<aios_apm_msgs::GetSlotIDCoordinates>("/aios/get_slot_id_coordinates");
    zone_info_srv = nh.serviceClient<aide_apm_msgs::GetZoneInfo>("/aide_query/zone_info");
    ontologyinfo_srv = nh.serviceClient<aios_apm_msgs::GetLaneInfo>("/aios/ontology_query/info");


    json_aios_req_apm_move  = AifoHandler::parseJsonFile("aios_req_apm_move",private_nh);
    json_aios_req_dest_arrived  = AifoHandler::parseJsonFile("aios_req_dest_arrived",private_nh);
    json_aios_req_logoff  = AifoHandler::parseJsonFile("aios_req_logoff",private_nh);
    json_aios_req_new_job  = AifoHandler::parseJsonFile("aios_req_new_job",private_nh);
    json_aios_req_poweroff  = AifoHandler::parseJsonFile("aios_req_poweroff",private_nh);
    json_aios_req_poweron  = AifoHandler::parseJsonFile("aios_req_poweron",private_nh);
    json_aios_req_update_trailer  = AifoHandler::parseJsonFile("aios_req_update_trailer",private_nh);
    json_aios_req_mode_change_update  = AifoHandler::parseJsonFile("aios_req_mode_change_update",private_nh);
    json_avcs_res_logon  = AifoHandler::parseJsonFile("avcs_res_logon",private_nh);
    json_hb_field_fault_req  = AifoHandler::parseJsonFile("hb_field_fault_req",private_nh);
    json_hb_field_pose_req  = AifoHandler::parseJsonFile("hb_field_pose_req",private_nh);
    json_hb_root_apm_to_avcs  = AifoHandler::parseJsonFile("hb_root_apm_to_avcs",private_nh);
    json_hb_root_avcs_to_apm  = AifoHandler::parseJsonFile("hb_root_avcs_to_apm",private_nh);
    json_aios_res_switch_mode  = AifoHandler::parseJsonFile("aios_res_switch_mode",private_nh);
    json_aios_res_job_response  = AifoHandler::parseJsonFile("aios_res_job_response",private_nh);
    json_avcs_req_job_instruction  = AifoHandler::parseJsonFile("avcs_req_job_instruction",private_nh);
    json_aios_res_refuel_response  = AifoHandler::parseJsonFile("aios_res_refuel_response",private_nh);
    json_avcs_req_refuel_instruction  = AifoHandler::parseJsonFile("avcs_req_refuel_instruction",private_nh);
    json_aios_res_maintenance_response  = AifoHandler::parseJsonFile("aios_res_maintenance_response",private_nh);
    json_avcs_req_maintenance_instruction  = AifoHandler::parseJsonFile("avcs_req_maintenance_instruction",private_nh);
    json_aios_res_park_response  = AifoHandler::parseJsonFile("aios_res_park_response",private_nh);
    json_avcs_req_park_instruction  = AifoHandler::parseJsonFile("avcs_req_park_instruction",private_nh);

    json_aios_res_path_update  = AifoHandler::parseJsonFile("aios_res_path_update",private_nh);
    json_avcs_req_path_update  = AifoHandler::parseJsonFile("avcs_req_path_update",private_nh);
    json_avcs_req_mi_req  = AifoHandler::parseJsonFile("avcs_req_mi_req",private_nh);
    json_avcs_req_linkup_req  = AifoHandler::parseJsonFile("avcs_req_linkup_req",private_nh);
    json_aios_res_linkup_res  = AifoHandler::parseJsonFile("aios_res_linkup_res",private_nh);

    json_aios_res_cancel_res = AifoHandler::parseJsonFile("aios_res_cancel_res",private_nh);
    json_avcs_req_dock = AifoHandler::parseJsonFile("avcs_req_dock",private_nh);
    json_aios_res_dock_res = AifoHandler::parseJsonFile("aios_res_dock_res",private_nh);
    json_aios_res_mount_res = AifoHandler::parseJsonFile("aios_res_mount_res",private_nh);
    json_aios_res_offload_res = AifoHandler::parseJsonFile("aios_res_offload_res",private_nh);
    json_avcs_req_mount = AifoHandler::parseJsonFile("avcs_req_mount",private_nh);
    json_avcs_req_offload = AifoHandler::parseJsonFile("avcs_req_offload",private_nh);
    json_avcs_req_stop_job_instruction = AifoHandler::parseJsonFile("avcs_req_stop_job_instruction",private_nh);
    json_aios_res_stop_job_res = AifoHandler::parseJsonFile("aios_res_stop_job_res",private_nh);

    json_avcs_req_resume_job_instruction = AifoHandler::parseJsonFile("avcs_req_resume_job_instruction",private_nh);
    json_aios_res_resume_job_res = AifoHandler::parseJsonFile("aios_res_resume_job_res",private_nh);
    json_avcs_req_manual_route_instruction = AifoHandler::parseJsonFile("avcs_req_manual_route_instruction",private_nh);
    json_aios_res_manual_route_response = AifoHandler::parseJsonFile("aios_res_manual_route_response",private_nh);
    json_avcs_req_manual_route_confirmation_request = AifoHandler::parseJsonFile("avcs_req_manual_route_confirmation_request",private_nh);
    json_aios_res_manual_route_confirmation_response = AifoHandler::parseJsonFile("aios_res_manual_route_confirmation_response",private_nh);

    json_empty_json = AifoHandler::parseJsonFile("empty_json",private_nh);





    try
    {
      json_avcs_converted_map_data  = AifoHandler::parseJsonFile("converted_map_csv_filepath",private_nh);
      json_avcs_reformatted_map_data  = AifoHandler::parseJsonFile("reformat_map_csv_filepath",private_nh);
    }
    catch(std::exception const& ex) {
      ROS_ERROR_STREAM("Missing map file");
    }
    // ROS_INFO_STREAM_COLOUR("cyan",json_avcs_converted_map_data);


    json_avcs_req_next_job_instruction = "";

    session_status_pub_ = nh.advertise<aios_apm_msgs::AifoStatus>(output_topic_session_status_, 1);

    debug_fr_container_pub_ = nh.advertise<std_msgs::Float32>("/viz/aios/avcs_handler/debug/cntr_fr_pos", 1);
    debug_bk_container_pub_ = nh.advertise<std_msgs::Float32>("/viz/aios/avcs_handler/debug/cntr_bk_pos", 1);
    debug_fr_container_id_pub_ = nh.advertise<std_msgs::String>("/viz/aios/avcs_handler/debug/cntr_fr_id", 1);
    debug_bk_container_id_pub_ = nh.advertise<std_msgs::String>("/viz/aios/avcs_handler/debug/cntr_bk_id", 1);
    debug_avcs_map_point_name_pub_ = nh.advertise<std_msgs::String>("/viz/aios/avcs_handler/point_name", 1);
    debug_avcs_opmode_pub_ = nh.advertise<std_msgs::String>("/viz/aios/avcs_handler/operation_mode", 1);
    debug_avcs_response_pub_ = nh.advertise<std_msgs::String>("/viz/aios/avcs_handler/response_str", 1);
    road_block_xya_pub_ = nh.advertise<aios_apm_msgs::OntologyPath>("/aios/avcs_handler/blocks", 1, true);
    speed_restrict_xya_pub_ = nh.advertise<aios_apm_msgs::OntologyPath>("/aios/avcs_handler/speed_restriction", 1, true);



    odom_msg_.header.frame_id = "pose_query";
    last_hb_time = ros::Time::now();

    json_response_history["response"] = nlohmann::json::array();

    JSON_AVCS_JOB_STATUS = 
    {
      {"AVCSJS_NOT_IN_OP", -1},  //OP_mode
      {"AVCSJS_WAITING_JOB_INSTRUCTION", 0},  //AJS 0, -1
      {"AVCSJS_WAITING_VESSEL_OPS_INFO", 1},
      {"AVCSJS_PROCESSING_JOB_INSTRUCTION", 2}, // AJS 1, 30 , 
      {"AVCSJS_EXECUTING_JOB_INSTRUCTION", 3}, // AJS 5, 7
      {"AVCSJS_SECOND_STAGE_ADJUSTMENT_FOR_JOB_INSTRUCTION", 4}, // AJS 
      {"AVCSJS_WAITING_DOCK_POSITION", 5}, // AJS 15 && activity
      {"AVCSJS_PROCESSING_DOCK_POSITION", 6}, 
      {"AVCSJS_EXECUTING_DOCK_POSITION", 7},
      {"AVCSJS_SECOND_STAGE_ADJUSTMENT_FOR_DOCK_POSITION", 8},
      {"AVCSJS_WAITING_ARMG_INSTRUCTION", 9},
      {"AVCSJS_SECOND_STAGE_ADJUSTMENT_FOR_ARMG", 10},
      {"AVCSJS_GENERATING_MA_ROUTE", 11}, // AJS 3 && job type
      {"AVCSJS_EXECUTING_MA_ROUTE", 12},
      {"AVCSJS_GENERATING_REFUEL_ROUTE", 13}, // AJS 3 && job type
      {"AVCSJS_EXECUTING_REFUEL_ROUTE", 14},
      {"AVCSJS_GENERATING_MANUAL_ROUTE", 15}, // AJS 3 && job type
      {"AVCSJS_EXECUTING_MANUAL_ROUTE", 16}, 
      {"AVCSJS_GENERATING_PARKING_ROUTE", 17}, // AJS 3 && job type
      {"AVCSJS_EXECUTING_PARKING_ROUTE", 18},
      {"AVCSJS_EXECUTING_STOP_ORDER", 19},
      {"AVCSJS_WAITING_INGRESS_TO_CALL_IN", 20},
      {"AVCSJS_PROCESSING_INGRESS_TO_CALL_IN", 21},
      {"AVCSJS_EXECUTING_INGRESS_TO_CALL_IN", 22},
      {"AVCSJS_WAITING_QC_REQUEST", 23},
      {"AVCSJS_PREPARING_QC_INFO_REQUEST", 24},
      {"AVCSJS_WAITING_QC_INFO", 25},
      {"AVCSJS_PROCESSING_QC_REQUEST", 26},
      {"AVCSJS_EXECUTING_MOVING_TO_QC", 27},
    };
}


  /************************************************************************************************************
  * Function to parse json files named from launch file to respective json variables
  ************************************************************************************************************/
  void AifoHandler::initialiseMqttComms()
  {
    json_aios_mqtt_config["type"] = "configuration";

    std::string client_id =  client_id_prefix + apm_id;
    json_aios_mqtt_config["client_id"] = client_id;
    json_aios_mqtt_config["address"] = broker_address;
    json_aios_mqtt_config["timeout"] = 1.0;
    json_aios_mqtt_config["sub_topic"] = nlohmann::json::array();
    json_aios_mqtt_config["pub_topic"] = nlohmann::json::array();

    nlohmann::json json_mqtt_subtopic;
    json_mqtt_subtopic["mqtt_sub_topic_name"] = "/" + client_id + mqtt_subtopic_instructions;
    json_mqtt_subtopic["publish_to_aios"] = aios_payload_intopic_instructions;
    json_aios_mqtt_config["sub_topic"].push_back(json_mqtt_subtopic);

    json_mqtt_subtopic["mqtt_sub_topic_name"] = mqtt_subtopic_maps;
    json_mqtt_subtopic["publish_to_aios"] = aios_payload_inttopic_maps;
    json_aios_mqtt_config["sub_topic"].push_back(json_mqtt_subtopic);

    json_mqtt_subtopic["mqtt_sub_topic_name"] = mqtt_subtopic_maps2;
    json_mqtt_subtopic["publish_to_aios"] = aios_payload_inttopic_maps;
    json_aios_mqtt_config["sub_topic"].push_back(json_mqtt_subtopic);

    json_mqtt_subtopic["mqtt_sub_topic_name"] = mqtt_subtopic_blocks;
    json_mqtt_subtopic["publish_to_aios"] = aios_payload_inttopic_maps;
    json_aios_mqtt_config["sub_topic"].push_back(json_mqtt_subtopic);

    nlohmann::json json_mqtt_pubtopic;
    json_mqtt_pubtopic["mqtt_pub_topic_name"] = "/" + client_id + mqtt_pubtopic_general;
    json_mqtt_pubtopic["get_from_aios"] = aios_payload_outttopic;
    json_aios_mqtt_config["pub_topic"].push_back(json_mqtt_pubtopic);

    

    std_msgs::String config_payload;
    config_payload.data = json_aios_mqtt_config.dump();
    avcs_requests_pub_.publish(config_payload);

    // ROS_ERROR_STREAM(json_aios_mqtt_config.dump(2));
  }


  /************************************************************************************************************
  * Function to parse json files named from launch file to respective json variables
  ************************************************************************************************************/
  nlohmann::json AifoHandler::parseJsonFile(std::string param_name, ros::NodeHandle private_nh)
  {
    std::string file_json_from_param;
    private_nh.getParam(param_name, file_json_from_param);
    std::ifstream ifs_json_file(file_json_from_param);
    return nlohmann::json::parse(ifs_json_file);

  }


  /************************************************************************************************************
  * Function to cast string to any type
  ************************************************************************************************************/
  template <class T>
  T stringToNum(std::string str)
  {
      T result;
      try
      {
          result = boost::lexical_cast<T>(str);
      }
    catch(std::exception const& ex) {
      ROS_WARN_STREAM("Can not cast" <<str<<", "<< ex.what());
          result = 0;
      }
      return result;
  }

    /************************************************************************************************************
    * Function to reset flags on power off
    ************************************************************************************************************/
    void AifoHandler::reset_flags_power()
    {
        avcs_status_msg.comms.power_off_requested = true;   
        avcs_status_msg.comms.power_on_requested = false;
        avcs_status_msg.fms_status.connectivity.fms_powered_on = false;
        avcs_status_msg.fms_status.connectivity.fms_trailer_updated = false;
        avcs_status_msg.fms_status.connectivity.fms_trailer_verified = false;
        avcs_status_msg.fms_status.connectivity.fms_trailer_id = "";
        avcs_status_msg.fms_status.operation.operation_mode = 0;
        avcs_status_msg.fms_status.operation.operation_mode_str = "MA";
    }




    /************************************************************************************************************
    * Function to reset flags on logoff or failed logon
    ************************************************************************************************************/
    void AifoHandler::reset_flags_logon()
    {
        avcs_status_msg.comms.HB_no_job = false;
        avcs_status_msg.job_info.fms_job_id = "";
        avcs_status_msg.job_info.fms_job_active_id = "";
        avcs_status_msg.job_info.local_job_id = 0;
        avcs_status_msg.job_info.fms_job_received_time = ros::Time();
        avcs_status_msg.fms_status.connectivity.fms_logged_on = false;
        avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in = false;
        avcs_status_msg.fms_status.connectivity.fms_trailer_verified = false;
        avcs_status_msg.fms_status.instructions.fms_instruction_job_type = 0;

        if(avcs_status_msg.fms_status.operation.operation_mode == 2)
        {
            avcs_status_msg.fms_status.operation.operation_mode = 1;
            avcs_status_msg.fms_status.operation.operation_mode_str = "TN";
        }
    }



    /************************************************************************************************************
    * Construct fault msg for AVCS HB
    ************************************************************************************************************/
    auto AifoHandler::construct_fault_message()
    {
        nlohmann::json json_req_msg = nlohmann::json::array();
        // if (apm_preprocess_msg.aisd_info.airs_sensors_action_response != 3)
        // {
        //     json_hb_field_fault_req["severity"] = "FATAL";
        //     json_hb_field_fault_req["code"] = 17;
        //     json_hb_field_fault_req["description"] = "airs_sensors_action_response: " + std::to_string(apm_preprocess_msg.aisd_info.airs_sensors_action_response);
        //     json_req_msg.push_back(json_hb_field_fault_req);
        // }
        // if (apm_preprocess_msg.aisd_info.aide_action_response != 3)
        // {
        //     json_hb_field_fault_req["severity"] = "FATAL";
        //     json_hb_field_fault_req["code"] = 17;
        //     json_hb_field_fault_req["description"] = "aide_action_response: " + std::to_string(apm_preprocess_msg.aisd_info.aide_action_response);
        //     json_req_msg.push_back(json_hb_field_fault_req);
        // }
        // if (apm_preprocess_msg.aisd_info.aipe_action_response != 3)
        // {
        //     json_hb_field_fault_req["severity"] = "FATAL";
        //     json_hb_field_fault_req["code"] = 17;
        //     json_hb_field_fault_req["description"] = "aipe_action_response: " + std::to_string(apm_preprocess_msg.aisd_info.aipe_action_response);
        //     json_req_msg.push_back(json_hb_field_fault_req);
        // }
        return json_req_msg;
    }
 
    /************************************************************************************************************
    * Construct localization msg for AVCS HB in lat lon
    ************************************************************************************************************/
    auto AifoHandler::construct_lc_message()
    {
        nlohmann::json json_req_msg;
        json_req_msg["convention"] = 0;

        json_req_msg["position"][0]=apm_preprocess_msg.aide_info.aide_lon_gl;
        json_req_msg["position"][1]=apm_preprocess_msg.aide_info.aide_lat_gl;
        json_req_msg["heading"] = apm_preprocess_msg.aide_info.current_pose_gl.theta;
        return json_req_msg;
    }
 
    /************************************************************************************************************
    * Construct root message to be sent to AVCS
    ************************************************************************************************************/
    auto AifoHandler::construct_root_message(nlohmann::json aios_req, nlohmann::json aios_res)
    {
      nlohmann::json json_req_msg;
      json_req_msg = json_hb_root_apm_to_avcs;
      json_req_msg["timestamp"]=static_cast<int64_t>(trunc((ros::Time::now().toNSec()/1000000)));
      json_req_msg["apm_id"]="APM" +(apm_preprocess_msg.platform_info.platform_id);

      if(avcs_status_msg.fms_status.operation.operation_mode == 0)
      {
          json_req_msg["operation_mode"]="MA";
          json_req_msg["apm_logon_status"] =0;
      }
      else if(avcs_status_msg.fms_status.operation.operation_mode == 1)
      {
          json_req_msg["operation_mode"]="TN";
          json_req_msg["apm_logon_status"] =0;
      }
      else if(avcs_status_msg.fms_status.operation.operation_mode == 2)
      {
          json_req_msg["operation_mode"]="OP";
        json_req_msg["apm_logon_status"] =1;
      }

      json_req_msg["mi_mode"] = apm_preprocess_msg.exception_info.exception_status != 0 ? "01" : 
      ((apm_preprocess_msg.airs_info.autonomous_status == 1)? "00" : "02");
      json_req_msg["apm_power_status"] = (avcs_status_msg.fms_status.connectivity.fms_powered_on) ? 1: 0;
      json_req_msg["faults"] = construct_fault_message();
      json_req_msg["fuel_level"] = fuel_level_avcs;
      json_req_msg["pose"] = construct_lc_message();
      json_req_msg["request_by_apm"] = aios_req;
      json_req_msg["response_by_apm"] = aios_res;
      json_req_msg["speed"] = apm_preprocess_msg.airs_info.speed_feedback/3.6;
      json_req_msg["ready_status"] = (avcs_status_msg.fms_status.operation.operation_mode == 2) ? 1 : 0;
      json_req_msg["manual_status"] = 0;
      
      if(apm_preprocess_msg.airs_info.gear_feedback == 1)
          json_req_msg["gear"] = "D";
      else if(apm_preprocess_msg.airs_info.gear_feedback == 2)
          json_req_msg["gear"] = "R";
      else
          json_req_msg["gear"] = "N";
	
    	json_req_msg["job_id"] = avcs_status_msg.job_info.fms_job_active_id;
    	json_req_msg["curr_block"] = apm_preprocess_msg.aide_info.yard_id;
    	json_req_msg["curr_lane"] = apm_preprocess_msg.aide_info.current_lane_str;
      json_req_msg["inside_yard_i"] = (apm_preprocess_msg.aide_info.current_lane_str !="")?"Y":"N";

      std::string direction_frame = "";
      try
      {
          direction_frame = static_cast<std::string>(frame_map_enum_list[apm_preprocess_msg.ontology_info.direction_frame]);
      }
      catch (const std::exception &e)
      {
          direction_frame = "";
      }

    	json_req_msg["direction"] = direction_frame;
    	json_req_msg["remaining_distance_to_destination"] = std::to_string(apm_preprocess_msg.aidc_info.aidc_trajectory_dist_to_end);

      return json_req_msg;
    }


    /************************************************************************************************************
    * Publish payload to MQTT client
    ************************************************************************************************************/
    auto AifoHandler::publish_payload(nlohmann::json root_msg)
    {
        std_msgs::String payload;
        payload.data = root_msg.dump();
        avcs_requests_pub_.publish(payload);
        ros::spinOnce();
    }


    /************************************************************************************************************
    * send PowerOff request and reset flags
    ************************************************************************************************************/
    bool AifoHandler::power_off_request()
    {
        ROS_DEBUG_STREAM("powering_off");     
        reset_flags_power();
        ROS_INFO_STREAM_COLOUR("green","****************** power_off_request*********************");
        ROS_INFO_STREAM_COLOUR("red",construct_root_message(json_aios_req_poweroff,json_empty_json));
        publish_payload(construct_root_message(json_aios_req_poweroff,json_empty_json));
        json_response_history["response"].push_back("Powering Off");
    }

    /************************************************************************************************************
    * send transition request
    * Only 2 modes allowed (MA and TN)
    ************************************************************************************************************/
    bool AifoHandler::transition_request(int mode)
    {
        boost::uuids::uuid session_uuid = boost::uuids::random_generator()();
        json_aios_req_mode_change_update["data"]["id"]= (boost::uuids::to_string(session_uuid));
        if (mode==0)
        {
          json_aios_req_mode_change_update["data"]["set_mode"]="MA";
          ROS_INFO_STREAM_COLOUR("green","Mode change to MA");
          avcs_status_msg.fms_status.operation.operation_mode = mode;  
          avcs_status_msg.fms_status.operation.operation_mode_str = "MA";  
          json_response_history["response"].push_back("Transition to MA");
        }
        else if (mode==1)
        {
         json_aios_req_mode_change_update["data"]["set_mode"]="TN";
          ROS_INFO_STREAM_COLOUR("green","Mode change to TN");
          avcs_status_msg.fms_status.operation.operation_mode = mode;  
          avcs_status_msg.fms_status.operation.operation_mode_str = "TN";  
          json_response_history["response"].push_back("Transition to TN");
        }

        ROS_INFO_STREAM_COLOUR("green","****************** transition_request *********************");
        ROS_INFO_STREAM_COLOUR("red",construct_root_message(json_aios_req_mode_change_update,json_empty_json));
        publish_payload(construct_root_message(json_aios_req_mode_change_update,json_empty_json));

    }

    /*****************************************************************************************************
    * send Job request for first time and only when in OP mode
    *****************************************************************************************************/
    auto AifoHandler::job_request()
    {
        if (avcs_status_msg.fms_status.operation.operation_mode != 2)
            return false;

        ROS_INFO_STREAM_COLOUR("green","****************** job_request *********************");
        ROS_INFO_STREAM_COLOUR("red",construct_root_message(json_aios_req_new_job,json_empty_json));
        publish_payload(construct_root_message(json_aios_req_new_job,json_empty_json));

        avcs_status_msg.job_progress.ready_for_new_job = true;

        // viz_avcs_response_msg.data = "Job request sent to AVCS - Ready for Autonomous Jobs";
        json_response_history["response"].push_back("Job request sent to AVCS - Ready for Autonomous Jobs");
        // debug_avcs_response_pub_.publish(viz_avcs_response_msg);
    }

    /*****************************************************************************************************
    * send apm arrive request
    *****************************************************************************************************/
    void AifoHandler::job_dest_reached()
    {
        json_response_history["response"].push_back("Destination reached sent to AVCS");
        // lanes_to_block.clear();
        // avcs_status_msg.fms_status.instructions.fms_job_received = false;
        json_aios_req_dest_arrived["data"]["id"] = avcs_status_msg.job_info.fms_job_active_id;
        avcs_status_msg.job_progress.reached_dest = true;
        ROS_ERROR_STREAM("****************** job_dest_reached *********************");
        publish_payload(construct_root_message(json_aios_req_dest_arrived,json_empty_json));
        ROS_ERROR_STREAM(construct_root_message(json_aios_req_dest_arrived,json_empty_json)<<std::endl);


        v2e_info_msg = aios_apm_msgs::V2eInfo(); 
        // viz_avcs_response_msg.data = "Destination reached sent to AVCS";
        // debug_avcs_response_pub_.publish(viz_avcs_response_msg);

  
    }


   

    /*****************************************************************************************************
    * Handle OP switch mode request by AVCS and send switch mode response back
    * Only 1 switch mode (to OP)
    * Only accept switch mode if in TN mode and reject if in MA mode or No trailer details
    *****************************************************************************************************/
    void AifoHandler::handle_avcs_op_request(nlohmann::json aifo_response_msg)
    {
        if(aifo_response_msg["request_by_avcs"]["data"]["set_mode"] == "OP")
        {
            if(avcs_status_msg.fms_status.operation.operation_mode == 0)
            {
                ROS_ERROR_STREAM("OP Not acknowledged");
                json_aios_res_switch_mode["data"]["success"] = 0;
                json_aios_res_switch_mode["data"]["rejection_code"] = "APM is in MA mode";
                json_aios_res_switch_mode["data"]["set_mode"] = "OP";
                avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in = false;                
                // viz_avcs_response_msg.data = "REJECT - APM is in MA mode";
                // debug_avcs_response_pub_.publish(viz_avcs_response_msg);
                json_response_history["response"].push_back("REJECT - APM is in MA mode");
            }
            else if(avcs_status_msg.fms_status.connectivity.fms_trailer_updated == false )
            {
                json_aios_res_switch_mode["data"]["success"] = 0;
                json_aios_res_switch_mode["data"]["rejection_code"] = "Trailer details not updated";
                json_aios_res_switch_mode["data"]["set_mode"] = "OP";
                avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in = false;              
                // viz_avcs_response_msg.data = "REJECT - Trailer details not updated";
                // debug_avcs_response_pub_.publish(viz_avcs_response_msg);
                json_response_history["response"].push_back("REJECT - Trailer details not updated");
            }
            else if(avcs_status_msg.fms_status.operation.operation_mode == 1)
            {
                json_aios_res_switch_mode["data"]["success"] = 1;
                json_aios_res_switch_mode["data"]["rejection_code"] = "";
                json_aios_res_switch_mode["data"]["set_mode"] = "OP";
                avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in = true;       
                // viz_avcs_response_msg.data = "SUCCESS - Switch to OP mode possible";
                // debug_avcs_response_pub_.publish(viz_avcs_response_msg);
                json_response_history["response"].push_back("SUCCESS - Switch to OP mode possible");
            }
            json_aios_res_switch_mode["data"]["id"] = aifo_response_msg["request_by_avcs"]["data"]["id"];
            ROS_INFO_STREAM_COLOUR("green","****************** switch_mode_response *********************");
            ROS_INFO_STREAM_COLOUR("red",construct_root_message(json_empty_json,json_aios_res_switch_mode));
            publish_payload(construct_root_message(json_empty_json,json_aios_res_switch_mode));
        }
    }

    auto AifoHandler::format_job_id(auto destination_label, int job_type)
    {
        ROS_ERROR_STREAM("destination_label: "<<destination_label);
        ROS_ERROR_STREAM("job_type: "<<job_type);


        std::string s = destination_label;
        std::string destination_label_aios = "";
        std::string delimiter = "_";

        if(job_type == 1 || job_type == 2 /*|| job_type == 4*/){
            size_t pos = 0;
            std::string token;
            int token_id = 1;

            while ((pos = s.find(delimiter)) != std::string::npos) {
                token = s.substr(0, pos);
                // std::cout << token << std::endl;
                s.erase(0, pos + delimiter.length());

                if(token_id == 1 )
                    destination_label_aios += token + "_";
                if(token_id == 3)
	            {
                	if(token == "0")
			             destination_label_aios += "00_";
			         else
			             destination_label_aios += "11_";
		        }
                token_id ++;
            }

            destination_label_aios += s;
            // destination_label_aios = destination_label_aios + ",20F_Front," + std::to_string(job_type);
        }
        else if(job_type == 3)
        {
            destination_label_aios = "RF_XX_";
            size_t pos = 0;
            std::string token;
            int token_id = 1;
            while ((pos = s.find(delimiter)) != std::string::npos) {
                token = s.substr(0, pos);
                // std::cout << token << std::endl;
                s.erase(0, pos + delimiter.length());

                if(token_id == 2){
                    destination_label_aios += token + "_";
                }
                token_id ++;
            }
            destination_label_aios += s;

        }
        // else if(job_type == 4 || job_type == 20)
        // {   
        
        //     destination_label_aios = "MBK";
        //     size_t pos = 0;
        //     std::string token;
        //     int token_id = 1;
        //     std::string block, slot;
        //     while ((pos = s.find(delimiter)) != std::string::npos) {
        //         token = s.substr(0, pos);
        //         ROS_ERROR_STREAM(token);
        //         s.erase(0, pos + delimiter.length());

               
        //         if(token_id == 1){
        //             block = token;
        //         }
        //         if(token_id == 2){
        //             slot = token;
        //         }
        //         token_id ++;
        //     }

        //     // destination_label_aios = block ;
        //     destination_label_aios += "_XX_" ;
        //     destination_label_aios += slot ;
        // }
        else if(job_type == 5 || job_type == 7 || job_type == 4 || job_type == 20)
        {
            std::string token_temp;
            std::string s_temp = destination_label;
            std::string delimiter = "_";
            token_temp = s_temp.substr(0, s_temp.find(delimiter));
            ROS_ERROR_STREAM("First element in destination_label : "<<token_temp);
            
            if (token_temp == "MB")
            {
                ROS_ERROR_STREAM("Received MB Job");
                destination_label_aios = "MBK";
                size_t pos = 0;
                std::string token;
                int token_id = 1;
                std::string block, slot;
                while ((pos = s.find(delimiter)) != std::string::npos) {
                    token = s.substr(0, pos);
                    // std::cout << token << std::endl;
                    ROS_ERROR_STREAM(token);
                    s.erase(0, pos + delimiter.length());

                    if(token_id == 1){
                        block = token;
                    }
                    // if(token_id == 2){
                    //     slot = token;
                    // }
                    //     destination_label_aios += token ;
                    //     destination_label_aios += "_XX_" ;
                    //     destination_label_aios += token ;
                    // }

                    token_id ++;
                }

                

                // destination_label_aios = block ;
                destination_label_aios += "_XX_" ;
                destination_label_aios += s ;
            }
            else{
                size_t pos = 0;
                std::string token;
                int token_id = 1;
                while ((pos = s.find(delimiter)) != std::string::npos) {
                    token = s.substr(0, pos);
                    // std::cout << token << std::endl;
                    s.erase(0, pos + delimiter.length());

                    if(token_id == 1 )
                        destination_label_aios += token + "_";
                    if(token_id == 3)
                    {
                        if(token == "0")
                             destination_label_aios += "00_";
                         else
                             destination_label_aios += "11_";
                    }
                    token_id ++;
                }
                
                destination_label_aios += s;
            }
        }
        
        destination_label_aios = destination_label_aios ;
        
        return destination_label_aios;
    }

    /*****************************************************************************************************
    * Handle Dock request, 
    * AVCS will send docking instructions to move to front cntr pos or rear cntr pos
    * First check if ready to dock,
    * if ready to dock, from the instructions check if docking required to front or back
    * 
    * 
    *        ----------   ----------
    *        *   2    *   *    4   *
    *        **********   **********
    *        ----------   ----------
    *  ////  *   1    *   *    3   *
    * /   /  **********   **********
    * /////////////////////////////////
    *    O               O    O
    *  
    *   
    *        -----------------------
    *        *          6          *
    *        ***********************
    *        -----------------------
    *  ////  *          5          *
    * /   /  ***********************
    * /////////////////////////////////
    *    O               O    O
    * 
    *   AIOS Docking config 1=> Front 2=> Rear
    *   Hence avcs config of 1,2,5,6 will be front and aios container config = 1
    *                   and  3,4     will be rear  and aios container config = 2
    *        ----------   ----------
    *  ////  *   1    *   *    2   *
    * /   /  **********   **********
    * /////////////////////////////////
    *    O               O    O
    *  
    *  
    * Copy Destination into response
    *****************************************************************************************************/


    /*****************************************************************************************************
    * Handle dock request, 
    * AVCS will send dock instructions
    *****************************************************************************************************/
    void AifoHandler::handle_dock_request(nlohmann::json aifo_response_msg)
    { 

        json_aios_res_dock_res["data"]["id"] = avcs_status_msg.job_info.fms_job_active_id;
        json_aios_req_apm_move["data"]["id"] = avcs_status_msg.job_info.fms_job_active_id;
        /////////////////////////// Handle Rejections ///////////////////////////////////
        // reject dock if not in OP
        if(!check_validity_condition(json_aios_res_dock_res, avcs_status_msg.fms_status.operation.operation_mode != 2, "01 - APM not in operation mode")) return;
        // reject dock if not in position
        if(!check_validity_condition(json_aios_res_dock_res, avcs_status_msg.job_progress.reached_dest != true, "17 - APM is not ready to handle dock position")) return;
        // Todo should check if in yard for docking
       
        /////////////////////////// Handle dock instruction  ///////////////////////////////////
        avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received = true;
        avcs_status_msg.job_info.activity = int(aifo_response_msg["request_by_avcs"]["data"]["activity"]);
        avcs_status_msg.job_info.aios_job_type = 8;

        local_container_config = 0;
        if (aifo_response_msg["request_by_avcs"]["data"]["cntr_location_on_apm"] <1 || aifo_response_msg["request_by_avcs"]["data"]["cntr_location_on_apm"] >6)
            local_container_config = 0;
        else
            local_container_config = (aifo_response_msg["request_by_avcs"]["data"]["cntr_location_on_apm"] == 3 ||
                                                            aifo_response_msg["request_by_avcs"]["data"]["cntr_location_on_apm"] == 4)? 2 : 1;

        
        avcs_status_msg.job_progress.ready_for_new_job = false;
        avcs_status_msg.job_progress.inprogress = true;
        avcs_status_msg.job_progress.container_information.container_to_mount =  aios_apm_msgs::ContainerDB();
        avcs_status_msg.job_progress.container_information.container_to_unmount =  aios_apm_msgs::ContainerDB();
        ROS_INFO_STREAM_COLOUR("green","****************** Dock Response *********************");
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_dock_res)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_dock_res));
        json_response_history["response"].push_back("Dock response sent");

        v2e_msg_count = 0;

        if(local_container_config != apm_preprocess_msg.session_info.current_dock_position)
        {
          ROS_ERROR_STREAM("****************** Dock Move timer *********************");
          timer = nh2.createTimer(ros::Duration(10.0), &AifoHandler::timerCallback2, this, true);
          timer2 = nh2.createTimer(ros::Duration(30.0), &AifoHandler::timerCallback3, this, true);
        }                
        else
        {
          ROS_ERROR_STREAM("****************** APM in position*********************");
          // job_dest_reached();

          timer = nh2.createTimer(ros::Duration(5.0), &AifoHandler::timerCallback, this, true);
        }        
    }

  void AifoHandler::timerCallback(const ros::TimerEvent &event)
  {
          job_dest_reached();
  }
  void AifoHandler::timerCallback2(const ros::TimerEvent &event)
  {
          ROS_ERROR_STREAM("****************** Dock Move*********************");
          ROS_ERROR_STREAM(construct_root_message(json_aios_req_apm_move,json_empty_json)<<std::endl);
          publish_payload(construct_root_message(json_aios_req_apm_move,json_empty_json));
          json_response_history["response"].push_back("Dock Move sent");
          avcs_status_msg.job_info.container_config = local_container_config;
  }
  void AifoHandler::timerCallback3(const ros::TimerEvent &event)
  {
          dock_move_requested = true;
  }
    /*****************************************************************************************************
    * Handle path update request, 
    * will relay the information from request to response with success as 1 until Wharf ops updated
    *****************************************************************************************************/
    void AifoHandler::handle_link_up_request(nlohmann::json aifo_response_msg)
    { 
        json_aios_res_linkup_res["data"]["id"] = aifo_response_msg["request_by_avcs"]["data"]["id"];
        std::string linkup_type = aifo_response_msg["request_by_avcs"]["data"]["link_up_type_c"];
        json_aios_res_linkup_res["data"]["link_up_type_c"] = linkup_type;
        json_aios_res_linkup_res["data"]["console_id"] = aifo_response_msg["request_by_avcs"]["data"]["console_id"];
        json_aios_res_linkup_res["data"]["ack_st_c"] = "00";

        json_aios_res_linkup_res["data"]["control_port_st_c"] = (linkup_type == "C") ? "L" : "NL";
        json_aios_res_linkup_res["data"]["view_port_st_c"] = (linkup_type == "V") ? "L" : "NL";
        json_aios_res_linkup_res["data"]["error_msg"] = "";


        ROS_ERROR_STREAM("****************** handle_link_up Response *********************");
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_linkup_res)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_linkup_res));

        json_response_history["response"].push_back("handle_link_up response sent");


        
    }
        
     /*****************************************************************************************************
    * Handle path update request, 
    * will relay the information from request to response with success as 1 until Wharf ops updated
    *****************************************************************************************************/
    void AifoHandler::handle_path_update_request(nlohmann::json aifo_response_msg)
    { 
        json_aios_res_path_update["data"]["id"] = aifo_response_msg["request_by_avcs"]["data"]["id"];
        json_aios_res_path_update["data"]["success"] = 1;
        json_aios_res_path_update["data"]["rejection_code"] = "";
        json_aios_res_path_update["data"]["current_route"] = nlohmann::json::array();
        
        if(aifo_response_msg["request_by_avcs"]["data"].contains("route_dag"))
        {
            
            for (const auto& item : aifo_response_msg["request_by_avcs"]["data"]["route_dag"].items())
            {
               if(item.value().contains("pose"))
                
                json_aios_res_path_update["data"]["current_route"].push_back(item.value()["pose"]);
               
            }

        }


        int n = aifo_response_msg["request_by_avcs"]["data"]["route_dag"].size();

        std::string destination_label_path_update;
        if(n>0)
          destination_label_path_update = aifo_response_msg["request_by_avcs"]["data"]["route_dag"][n-1]["name"];


        // json_aios_res_path_update["data"]["dest_location"] = aifo_response_msg["request_by_avcs"]["data"]["destination_waypoint"];
        json_aios_res_path_update["data"]["dest_location"] = destination_label_path_update;
        json_aios_res_path_update["data"]["destination_name"] = aifo_response_msg["request_by_avcs"]["data"]["destination_name"];
        json_aios_res_path_update["data"]["affected_by_road_blocks"] = 0;


        ROS_ERROR_STREAM("****************** Path Update Response *********************");
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_path_update)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_path_update));

        json_response_history["response"].push_back("Path Update response sent");


    }
       
    /*****************************************************************************************************
    * Handle Mount / Offload request, 
    * AVCS will send Mount / Offload instructions to update front cntr pos or rear cntr pos
    * Check cntr pos and update db
    *****************************************************************************************************/
    void AifoHandler::handle_mount_request(nlohmann::json aifo_response_msg)
    { 
        json_aios_res_mount_res["data"]["id"] = avcs_status_msg.job_info.fms_job_active_id;
        bool fr_occupied = avcs_status_msg.job_progress.container_information.cntr_fr_pos_occupied;
        bool bk_occupied = avcs_status_msg.job_progress.container_information.cntr_bk_pos_occupied;
        int request_cntr_pos =  aifo_response_msg["request_by_avcs"]["data"]["cntr_location_on_apm"];
        std::string request_cntr_id =  aifo_response_msg["request_by_avcs"]["data"]["cntr_number"];

        /////////////////////////// Handle Rejections ///////////////////////////////////
        // reject if not docked and APM is not ready to receive mount instruction
        if(!check_validity_condition(json_aios_res_mount_res, 
          (!avcs_status_msg.job_progress.reached_dest/*|| !apm_preprocess_msg.session_info.job_ready_to_be_served*/),
           "06 - APM is not ready to receive mount instruction")) return;
       
        // reject if pos occupied unless overide allowed
        if(!check_validity_condition(json_aios_res_mount_res, 
          ((!param_container_overide && fr_occupied && ( request_cntr_pos== 1 || request_cntr_pos== 5)) ||
            (!param_container_overide && bk_occupied && ( request_cntr_pos== 3 || request_cntr_pos== 5))),
           "08 - Container already mounted on target mounting position")) return;
       

        avcs_status_msg.fms_status.instructions.fms_job_mount_cmd_received = true;
        avcs_status_msg.job_progress.mounting = true;
        avcs_status_msg.job_progress.offloading = false;

        json_aios_res_mount_res["data"]["success"] = 1;
        json_aios_res_mount_res["data"]["rejection_code"] = "";
        ROS_ERROR_STREAM("****************** Mount Response *********************");
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_mount_res)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_mount_res));

        json_response_history["response"].push_back("Mount response sent");
        avcs_status_msg.job_progress.ready_for_new_job = true;

        if(!fr_occupied && ( request_cntr_pos== 1 || request_cntr_pos== 5))
        {
            avcs_status_msg.job_progress.container_information.cntr_fr_pos_occupied = true;
            fr_contr_pos.data = 1.0;
            fr_contr_id.data = request_cntr_id;
        }

        if(!bk_occupied && ( request_cntr_pos== 3 || request_cntr_pos== 5))
        {
            avcs_status_msg.job_progress.container_information.cntr_bk_pos_occupied = true;
            bk_contr_pos.data = 1.0;
            bk_contr_id.data = request_cntr_id;
        }

        aios_apm_msgs::ContainerDB container_info;
        container_info.container_id = request_cntr_id;
        container_info.container_size = stringToNum<int>(aifo_response_msg["request_by_avcs"]["data"]["cntr_size"]);
        container_info.container_mount_pos_on_apm = request_cntr_pos;
        ROS_ERROR_STREAM(container_info);
        avcs_status_msg.job_progress.container_information.mounted_containers.push_back(container_info);
        avcs_status_msg.job_progress.container_information.container_to_mount = container_info;
        avcs_status_msg.job_progress.inprogress = false;
    }
  

    /*****************************************************************************************************
    * Handle Mount / Offload request, 
    * AVCS will send Mount / Offload instructions to update front cntr pos or rear cntr pos
    * Check cntr pos and update db
    *****************************************************************************************************/   
    void AifoHandler::handle_offload_request(nlohmann::json aifo_response_msg)
    { 
        json_aios_res_offload_res["data"]["id"] = avcs_status_msg.job_info.fms_job_active_id;
        bool fr_occupied = avcs_status_msg.job_progress.container_information.cntr_fr_pos_occupied;
        bool bk_occupied = avcs_status_msg.job_progress.container_information.cntr_bk_pos_occupied;
        // int request_cntr_pos =  aifo_response_msg["request_by_avcs"]["data"]["cntr_location_on_apm"];
        std::string request_cntr_id =  aifo_response_msg["request_by_avcs"]["data"]["cntr_number"];


        aios_apm_msgs::ContainerDB container_info;
        container_info.container_id = request_cntr_id;
        // container_info.container_size = stringToNum<int>(aifo_response_msg["request_by_avcs"]["data"]["cntr_size"]);
        // container_info.container_mount_pos_on_apm = request_cntr_pos;
        avcs_status_msg.job_progress.container_information.container_to_unmount = container_info;

        /////////////////////////// Handle Rejections ///////////////////////////////////
        // reject if not docked and APM is not ready to receive offload instruction
        if(!check_validity_condition(json_aios_res_offload_res, 
          (!avcs_status_msg.job_progress.reached_dest/*|| !apm_preprocess_msg.session_info.job_ready_to_be_served*/),
           "09 - APM is not ready to receive offload instruction")) return;
       
        bool container_in_db = false;
        for (auto it = avcs_status_msg.job_progress.container_information.mounted_containers.begin(); 
            it != avcs_status_msg.job_progress.container_information.mounted_containers.end(); ) 
        {
            if (it->container_id == request_cntr_id) {
                container_in_db = true;
                avcs_status_msg.fms_status.instructions.fms_job_offload_cmd_received = true;
                avcs_status_msg.job_progress.offloading = true;
                avcs_status_msg.job_progress.mounting = false;
                json_aios_res_offload_res["data"]["success"] = 1;
                json_aios_res_offload_res["data"]["rejection_code"] = "";
                ROS_ERROR_STREAM("****************** OFFLOAD Response *********************");
                ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_offload_res)<<std::endl);
                publish_payload(construct_root_message(json_empty_json,json_aios_res_offload_res));
                json_response_history["response"].push_back("Offload response sent");
                avcs_status_msg.job_progress.ready_for_new_job = true;
                // avcs_status_msg.job_info = aios_apm_msgs::JobInfo();

                if(fr_occupied && ( it->container_mount_pos_on_apm == 1 || it->container_mount_pos_on_apm == 5))
                {
                    avcs_status_msg.job_progress.container_information.cntr_fr_pos_occupied = false;
                    fr_contr_pos.data = 0.0;
                    fr_contr_id.data = "Empty";
                }

                if(bk_occupied == true && ( it->container_mount_pos_on_apm == 3 || it->container_mount_pos_on_apm == 5))
                {
                    avcs_status_msg.job_progress.container_information.cntr_bk_pos_occupied = false;
                    bk_contr_pos.data = 0.0;
                    bk_contr_id.data = "Empty";
                }

                it = avcs_status_msg.job_progress.container_information.mounted_containers.erase(it);
                avcs_status_msg.job_progress.inprogress = false;
                break;
            } else {
                ++it;
            }
        }
 
        // reject if no container id in DB
        if(!check_validity_condition(json_aios_res_offload_res, !container_in_db, "12 - APM Container database is wrong, can't accept offload instruction")) return;        
    }



    /*****************************************************************************************************
    * Reset job progress types
    *****************************************************************************************************/
    void AifoHandler::resetJobProgressOnNewInstructions()
    { 
      avcs_status_msg.fms_status.instructions.fms_job_offload_cmd_received = false;
      avcs_status_msg.fms_status.instructions.fms_job_mount_cmd_received = false;
      avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received = false;
      avcs_status_msg.fms_status.instructions.fms_responded =false;
      avcs_status_msg.fms_status.instructions.fms_release_nav =false;
      avcs_status_msg.job_progress.reached_dest = false;
      avcs_status_msg.job_progress.mounting = false;
      avcs_status_msg.job_progress.offloading = false;
      path_received_from_session = false;

      avcs_status_msg.job_progress.container_information.container_to_mount =  aios_apm_msgs::ContainerDB();
      avcs_status_msg.job_progress.container_information.container_to_unmount =  aios_apm_msgs::ContainerDB();
    }


    /*****************************************************************************************************
    * Function to publish rejection for instructions
    *****************************************************************************************************/
    bool AifoHandler::check_validity_condition(nlohmann::json json_aios_res_xx_response, bool reject_condition_met, std::string reject_msg)
    {
        if(reject_condition_met)
        {
            json_aios_res_xx_response["data"]["success"] = 0;
            json_aios_res_xx_response["data"]["rejection_code"] = reject_msg;
            ROS_ERROR_STREAM("****************** REJECT request : "<<reject_msg <<" *********************");
            ROS_INFO_STREAM_COLOUR("yellow",construct_root_message(json_empty_json,json_aios_res_xx_response));
            publish_payload(construct_root_message(json_empty_json,json_aios_res_xx_response));

            json_response_history["response"].push_back("REJECT request : " + reject_msg);
            return false;
        }
        else
            return true;
    }


    /*****************************************************************************************************
    * Retrieve job info details from instructions and return
    *****************************************************************************************************/
    aios_apm_msgs::JobInfo AifoHandler::retriveJobInfoFromInstruction(nlohmann::json avcs_instruction_msg, int instruction_type, bool precheck)
    {
        ROS_INFO_STREAM("retriveJobInfoFromInstruction: "<<instruction_type);
        aios_apm_msgs::JobInfo job_info_avcs;
        int n = avcs_instruction_msg["data"]["route_dag"].size();
        job_info_avcs.no_of_given_points = n;
        if(n < 1)
          return job_info_avcs;
        job_info_avcs.new_job = true;
        job_info_avcs.fms_job_received_time = ros::Time::now();
        job_info_avcs.fms_job_id = avcs_instruction_msg["data"]["id"];
        job_info_avcs.fms_job_active_id = avcs_instruction_msg["data"]["id"];
        job_info_avcs.local_job_id = local_job_id;

        // job_info_avcs.local_job_id = 1;
        std::string destination_label_avcs = avcs_instruction_msg["data"]["route_dag"][n-1]["name"];
        job_info_avcs.fms_job_destination_id = destination_label_avcs;

        ROS_INFO_STREAM_COLOUR("yellow",destination_label_avcs);

        if(instruction_type == 1){
          job_info_avcs.activity = int(avcs_instruction_msg["data"]["activity"]) ;
          job_info_avcs.container_type = avcs_instruction_msg["data"]["assigned_cntr_size"];
          job_info_avcs.container_config = (avcs_instruction_msg["data"]["target_dock_position"] == "3" ||
                                                        avcs_instruction_msg["data"]["target_dock_position"] == "4")? 2 :
                                                        (avcs_instruction_msg["data"]["target_dock_position"] == "")? 0 : 1;

        }
        else if(instruction_type >= 10){
          // job_info_avcs.activity = 0;
          job_info_avcs.container_config = (avcs_instruction_msg["data"]["cntr_location_on_apm"] == "3" ||
                                                        avcs_instruction_msg["data"]["cntr_location_on_apm"] == "4")? 2 :
                                                        (avcs_instruction_msg["data"]["cntr_location_on_apm"] == "")? 0 : 1;

          job_info_avcs.container_type = avcs_instruction_msg["data"]["cntr_size"];
          job_info_avcs.activity = avcs_status_msg.job_info.activity;
          if(job_info_avcs.activity == 3 || job_info_avcs.activity  == 7)
            job_info_avcs.activity =0;
          

        }
        else {
          job_info_avcs.activity = 0;

        }
        job_info_avcs.aios_job_type = (job_info_avcs.activity == 2) ? 1 :  //loading
                                                 (job_info_avcs.activity == 6) ? 2 : //offloading
                                                 (job_info_avcs.activity == 1) ? 5 : //standby
                                                 (job_info_avcs.activity == 5) ? 7 : //endslot
                                                 (job_info_avcs.activity == 3) ? 1 :
                                                 (job_info_avcs.activity == 7) ? 2 :
                                                 (instruction_type % 10 == 4) ? 3 : //refuel
                                                 (instruction_type % 10 == 3) ? 4 : //parking
                                                 (instruction_type % 10 == 2) ? 20 : 5;   //20- maintainence    // anything else, retain in standby job
                                                                                       //INSTRUCTION TYPE:  2- mantenance, 3- Parking, 4- refuelling
        
        if( instruction_type % 10 == 2 && json_avcs_reformatted_map_data[destination_label_avcs]["type"]  != "maintenance_spot")
          job_info_avcs.aios_job_type = 5;
        if( instruction_type % 10 == 3 && json_avcs_reformatted_map_data[destination_label_avcs]["type"]  != "parking_spot")
          job_info_avcs.aios_job_type = 5;
        // if( job_info_avcs.aios_job_type == 4 && json_avcs_reformatted_map_data[destination_label_avcs]["type"]  != "refueling_spot")
        //   job_info_avcs.aios_job_type = 5;

        job_info_avcs.aios_job_location_id = format_job_id(destination_label_avcs,job_info_avcs.aios_job_type);
        job_info_avcs.given_route_mandatory = (instruction_type >= 10 || avcs_instruction_msg["data"]["route_mandate"] == "Y")? true : false;

        if(avcs_instruction_msg["data"]["route_dag"][n-1]["pose"]["position"].size() < 2 )
        {
          ROS_ERROR_STREAM("Position size not met. not displaying provided path");
          return job_info_avcs;
        }

        if(avcs_instruction_msg["data"]["route_dag"][n-1]["pose"]["convention"] == 10)
        {
          job_info_avcs.fms_job_destination_utm.x = avcs_instruction_msg["data"]["route_dag"][n-1]["pose"]["position"][0];
          job_info_avcs.fms_job_destination_utm.y = avcs_instruction_msg["data"]["route_dag"][n-1]["pose"]["position"][1];
        }
        else{
          double dst_northing_, dst_easting_;
          job_info_avcs.fms_job_destination.x = avcs_instruction_msg["data"]["route_dag"][n-1]["pose"]["position"][0];
          job_info_avcs.fms_job_destination.y = avcs_instruction_msg["data"]["route_dag"][n-1]["pose"]["position"][1];    
          gps_common::UTM(job_info_avcs.fms_job_destination.y, job_info_avcs.fms_job_destination.x, &dst_easting_ , &dst_northing_);
          job_info_avcs.fms_job_destination_utm.x = dst_easting_;
          job_info_avcs.fms_job_destination_utm.y = dst_northing_;    
        }
        
        double squared_dist = pow(job_info_avcs.fms_job_destination_utm.x-apm_preprocess_msg.aide_info.current_pose_gl.x,2) + 
                              pow(job_info_avcs.fms_job_destination_utm.y-apm_preprocess_msg.aide_info.current_pose_gl.y,2);

        job_info_avcs.distance2destination = sqrt(squared_dist);
        if(precheck)
          return job_info_avcs;




        pcl::PointCloud<pcl::PointXYZI>::Ptr given_path_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        given_path_cloud->header.frame_id = "utm";


        manual_path_trajectory.ca.clear();
        int path_size = avcs_instruction_msg["data"]["route_dag"].size();
        int index = 0;
        bool utm = false;
        std::string current_lane_id = retrieve_lane_id(apm_preprocess_msg.aide_info.current_pose_bl);
        std::string dest_lane_id;
        ROS_INFO_STREAM_COLOUR("yellow",current_lane_id);

        for (const auto& item : avcs_instruction_msg["data"]["route_dag"].items())
        {
            double p_northing_, p_easting_;
            if(item.value()["pose"]["convention"] != 10)
              gps_common::UTM(item.value()["pose"]["position"][1], item.value()["pose"]["position"][0], &p_easting_ , &p_northing_);
            else    
            {          
              p_easting_ = item.value()["pose"]["position"][0];
              p_northing_ = item.value()["pose"]["position"][1];
              utm = true;
            }

            pcl::PointXYZI p;
            p.intensity = 50;
            std::string road_name = item.value()["name"];
            // ROS_INFO_STREAM_COLOUR("blue", json_avcs_reformatted_map_data[road_name]["type"]);


            std::string closest_point_type = json_avcs_reformatted_map_data[closest_point_name.data]["type"];
            if(index == 0 &&  closest_point_type != "bypass" && closest_point_type != "yard"/*&& apm_preprocess_msg.aide_info.current_lane_str == ""*/)
            {
              pcl::PointXYZI p_start;
              p_start.intensity = 10;
              job_info_avcs.given_route_x.push_back(apm_preprocess_msg.aide_info.current_pose_gl.x);
              job_info_avcs.given_route_y.push_back(apm_preprocess_msg.aide_info.current_pose_gl.y);
              job_info_avcs.given_route_a.push_back(1.0);
              manual_path_trajectory.ca.push_back(1.0);
              p_start.x = apm_preprocess_msg.aide_info.current_pose_gl.x;
              p_start.y = apm_preprocess_msg.aide_info.current_pose_gl.y;
              p_start.z = 1;
              given_path_cloud->push_back(p_start);
            }


            if((json_avcs_reformatted_map_data[road_name]["type"] != "bypass" && json_avcs_reformatted_map_data[road_name]["type"] != "yard" )
            || (road_name.find("bypass_bypass_stop") != std::string::npos)
            || (road_name.find("bypass_right_stop") != std::string::npos)
            || (road_name.find("bypass_left_stop") != std::string::npos)
            || (json_avcs_reformatted_map_data[road_name]["type"] == "yard" && index == path_size -1  ))
            {
              job_info_avcs.given_route_x.push_back(p_easting_);
              job_info_avcs.given_route_y.push_back(p_northing_);
              job_info_avcs.given_route_a.push_back(1.0);
              manual_path_trajectory.ca.push_back(1.0);
              p.intensity = 100;
            }

            if(index == path_size -1 )
            {
              geometry_msgs::Pose2D dest_route_point;
              dest_route_point.x = p_easting_;
              dest_route_point.y = p_northing_;
              dest_lane_id = retrieve_lane_id(dest_route_point);
              ROS_INFO_STREAM_COLOUR("yellow",dest_lane_id);

              if(dest_lane_id == current_lane_id && job_info_avcs.given_route_x.size() == 1)
              {
                job_info_avcs.given_route_x.insert(job_info_avcs.given_route_x.begin(), apm_preprocess_msg.aide_info.current_pose_gl.x);
                job_info_avcs.given_route_y.insert(job_info_avcs.given_route_y.begin(), apm_preprocess_msg.aide_info.current_pose_gl.y);
              }

            }
            
            p.x = p_easting_;
            p.y = p_northing_;
            p.z = 1;
            given_path_cloud->push_back(p);

            index++;
        }

        manual_path_trajectory.cx = job_info_avcs.given_route_x;
        manual_path_trajectory.cy = job_info_avcs.given_route_y;


        // ROS_ERROR_STREAM("squared_dist ----------------: " << squared_dist);
        // if( squared_dist < param_acceptable_squared_distance_from_current)
        // {
        //   aios_apm_msgs::JobInfo job_info_avcs;
        //   return job_info_avcs;
        // }

        given_route_pcl_pub_.publish(given_path_cloud);
        return job_info_avcs;
    }

    /*****************************************************************************************************
    * Handle maintenance request, 
    * AVCS will send maintenance instructions
    * Copy Destination into response
    *****************************************************************************************************/
    void AifoHandler::handle_avcs_maintenance_request(nlohmann::json aifo_response_msg)
    { 
        json_avcs_req_maintenance_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_maintenance_response["data"]["current_route"] = nlohmann::json::array();
        json_aios_res_maintenance_response["data"]["id"] = json_avcs_req_maintenance_instruction["data"]["id"];
        int instruction_job_type = 2;
        next_road_block_instruction_job_type = instruction_job_type;
        json_avcs_road_blocked_next_job_instruction = aifo_response_msg;
        bool overide_same_type_job = avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == instruction_job_type;



        /////////////////////////// Handle Rejections ///////////////////////////////////
        // reject park if not in OP
        if(!check_validity_condition(json_aios_res_maintenance_response, avcs_status_msg.fms_status.operation.operation_mode != 2, "01 - APM not in operation mode")) return;
        // reject park if not ready for new job

        if(!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job)
        {
            request_job_trigger = true;
             if(!check_validity_condition(json_aios_res_maintenance_response, (!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job), "15 - APM is doing another job")) return;
        }

        // if(!check_validity_condition(json_aios_res_maintenance_response, (!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job), "15 - APM is doing another job")) return;
        // reject park if mounted with container
        if(!check_validity_condition(json_aios_res_maintenance_response, avcs_status_msg.job_progress.container_information.mounted_containers.size() != 0, "xx - APM currently have mounted container")) return;
        // reject park if fuel less than 20%
        if(!check_validity_condition(json_aios_res_maintenance_response, fuel_level_avcs < 20, "xx - Low Fuel")) return;


        /////////////////////////// Handle maintenance job  ///////////////////////////////////
        if(avcs_status_msg.fms_status.instructions.fms_job_received != true)
        {
            ROS_INFO_STREAM("Handling MAINTENANCE");
            json_avcs_req_next_job_instruction = "";            
            next_instruction_job_type = 0;

            /////////////////////////// Update instruction status ///////////////////////////////////
            avcs_status_msg.fms_status.instructions.fms_instruction_job_type = instruction_job_type;
            avcs_status_msg.fms_status.instructions.fms_job_received = true;

            /////////////////////////// Update job info ///////////////////////////////////
            avcs_status_msg.job_info = retriveJobInfoFromInstruction(json_avcs_req_maintenance_instruction,instruction_job_type);

            bool near_dest = avcs_status_msg.job_info.distance2destination > 0 && avcs_status_msg.job_info.distance2destination < param_acceptable_distance_from_current;
            if(!check_validity_condition(json_aios_res_maintenance_response, avcs_status_msg.job_info.fms_job_active_id == "", "107 - Couldnt resolve job id.")) return;
            if(!check_validity_condition(json_aios_res_maintenance_response, near_dest , "57 - Destination close - sending arrive"))
            {
              job_dest_reached();
              return;
            }

            /////////////////////////// Update response message ///////////////////////////////////

            json_aios_res_maintenance_response["data"]["success"] = 1;
            json_aios_res_maintenance_response["data"]["rejection_code"] = "";

            resetJobProgressOnNewInstructions();

            // TODO This should be moved to after session verified
            avcs_status_msg.job_progress.ready_for_new_job = false;
        }
        else if(avcs_status_msg.job_progress.ready_for_new_job == true || overide_same_type_job){            
            aios_apm_msgs::JobInfo job_info_temp =  retriveJobInfoFromInstruction(json_avcs_req_maintenance_instruction,instruction_job_type,true);
            if(job_info_temp.distance2destination > 0 && job_info_temp.distance2destination < param_acceptable_distance_from_current)
            {
              avcs_status_msg.job_info.fms_job_active_id = job_info_temp.fms_job_id;
              json_response_history["response"].push_back("Destination close - sending arrive");
              job_dest_reached();
              return;
            }
            ROS_INFO_STREAM("Accepting next instruction");
            json_avcs_req_next_job_instruction = aifo_response_msg;
            complete_job_for_new = true;
            next_instruction_job_type = 2;
        }

    }

    /*****************************************************************************************************
    * Handle parking request,
    * AVCS will send parking instructions
    * Copy path into response current_route(Temp)
    * Copy Destination into response
    *****************************************************************************************************/
    void AifoHandler::handle_avcs_parking_request(nlohmann::json aifo_response_msg)
    {
        json_avcs_req_park_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_park_response["data"]["current_route"] = nlohmann::json::array();
        json_aios_res_park_response["data"]["id"] = json_avcs_req_park_instruction["data"]["id"];
        int instruction_job_type = 3;
        next_road_block_instruction_job_type = instruction_job_type;
        json_avcs_road_blocked_next_job_instruction = aifo_response_msg;
        bool overide_same_type_job = avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == instruction_job_type;


        /////////////////////////// Handle Rejections ///////////////////////////////////
        // reject park if not in OP
        if(!check_validity_condition(json_aios_res_park_response, avcs_status_msg.fms_status.operation.operation_mode != 2, "01 - APM not in operation mode")) return;
        // reject park if not ready for new job

        if(!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job)
        {
            request_job_trigger = true;
             if(!check_validity_condition(json_aios_res_park_response, (!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job), "15 - APM is doing another job")) return;
        }

        // if(!check_validity_condition(json_aios_res_park_response, (!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job), "15 - APM is doing another job")) return;
        // // reject park if mounted with container
        if(!check_validity_condition(json_aios_res_park_response, avcs_status_msg.job_progress.container_information.mounted_containers.size() != 0, "xx - APM currently have mounted container")) return;
        // reject park if fuel less than 20%
        if(!check_validity_condition(json_aios_res_park_response, fuel_level_avcs < 20, "xx - Low Fuel")) return;

        /////////////////////////// Handle parking job  ///////////////////////////////////
        if(avcs_status_msg.fms_status.instructions.fms_job_received != true)
        {
            ROS_INFO_STREAM("Handling PARKING");
            json_avcs_req_next_job_instruction = "";          
            next_instruction_job_type = 0;

            /////////////////////////// Update instruction status ///////////////////////////////////
            avcs_status_msg.fms_status.instructions.fms_instruction_job_type =instruction_job_type;
            avcs_status_msg.fms_status.instructions.fms_job_received = true;

            /////////////////////////// Update job info ///////////////////////////////////
            avcs_status_msg.job_info = retriveJobInfoFromInstruction(json_avcs_req_park_instruction,instruction_job_type);

            bool near_dest = avcs_status_msg.job_info.distance2destination > 0 && avcs_status_msg.job_info.distance2destination < param_acceptable_distance_from_current;
            if(!check_validity_condition(json_aios_res_park_response, avcs_status_msg.job_info.fms_job_active_id == "", "107 - Couldnt resolve job id.")) return;
            if(!check_validity_condition(json_aios_res_park_response, near_dest , "57 - Destination close - sending arrive")) 
            {
              job_dest_reached();
              return;
            }


            /////////////////////////// Update response message ///////////////////////////////////
            json_aios_res_park_response["data"]["success"] = 1;
            json_aios_res_park_response["data"]["rejection_code"] = "";

            resetJobProgressOnNewInstructions();

            // TODO This should be moved to after session verified
            avcs_status_msg.job_progress.ready_for_new_job = false;
        }
        else if(avcs_status_msg.job_progress.ready_for_new_job == true || overide_same_type_job){          
            aios_apm_msgs::JobInfo job_info_temp =  retriveJobInfoFromInstruction(json_avcs_req_park_instruction,instruction_job_type,true);
            if(job_info_temp.distance2destination > 0 && job_info_temp.distance2destination < param_acceptable_distance_from_current)
            {
              avcs_status_msg.job_info.fms_job_active_id = job_info_temp.fms_job_id;
              json_response_history["response"].push_back("Destination close - sending arrive");
              job_dest_reached();
              return;
            }
            ROS_INFO_STREAM("Accepting next instruction");
            json_avcs_req_next_job_instruction = aifo_response_msg;
            complete_job_for_new = true;
            next_instruction_job_type = 3;
        }

    }
    
    /*****************************************************************************************************
    * Handle refuel request, 
    * AVCS will send refuel instructions
    * Copy Destination into response
    *****************************************************************************************************/
    void AifoHandler::handle_avcs_refuel_request(nlohmann::json aifo_response_msg)
    { 
        json_avcs_req_refuel_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_refuel_response["data"]["current_route"] = nlohmann::json::array() ;
        json_aios_res_refuel_response["data"]["id"] = json_avcs_req_refuel_instruction["data"]["id"];
        int instruction_job_type = 4;
        next_road_block_instruction_job_type = instruction_job_type;
        json_avcs_road_blocked_next_job_instruction = aifo_response_msg;
        bool overide_same_type_job = avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == instruction_job_type;

        /////////////////////////// Handle Rejections ///////////////////////////////////
        // reject refuel if not in OP
        if(!check_validity_condition(json_aios_res_refuel_response, avcs_status_msg.fms_status.operation.operation_mode != 2, "01 - APM not in operation mode")) return;
        // reject refuel if not ready for new job
        if(!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job)
        {
            request_job_trigger = true;
            if(!check_validity_condition(json_aios_res_refuel_response, (!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job), "15 - APM is doing another job")) return;
        }

        // if(!check_validity_condition(json_aios_res_refuel_response, (!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job), "15 - APM is doing another job")) return;
        // reject refuel if mounted with container
        if(!check_validity_condition(json_aios_res_refuel_response, avcs_status_msg.job_progress.container_information.mounted_containers.size() != 0, "xx - APM currently have mounted container")) return;


        /////////////////////////// Handle refuel job  ///////////////////////////////////
        if(avcs_status_msg.fms_status.instructions.fms_job_received != true)
        {
            ROS_INFO_STREAM("Handling REFUEL");
            json_avcs_req_next_job_instruction = "";          
            next_instruction_job_type = 0;

            /////////////////////////// Update instruction status ///////////////////////////////////
            avcs_status_msg.fms_status.instructions.fms_instruction_job_type = instruction_job_type;
            avcs_status_msg.fms_status.instructions.fms_job_received = true;        

            /////////////////////////// Update job info ///////////////////////////////////
            avcs_status_msg.job_info = retriveJobInfoFromInstruction(json_avcs_req_refuel_instruction,instruction_job_type);

            bool near_dest = avcs_status_msg.job_info.distance2destination > 0 && avcs_status_msg.job_info.distance2destination < param_acceptable_distance_from_current;
            if(!check_validity_condition(json_aios_res_refuel_response, avcs_status_msg.job_info.fms_job_active_id == "", "107 - Couldnt resolve job id.")) return;
            if(!check_validity_condition(json_aios_res_refuel_response, near_dest , "57 - Destination close - sending arrive")) 
            {
              job_dest_reached();
              return;
            }

            /////////////////////////// Update response message ///////////////////////////////////
            json_aios_res_refuel_response["data"]["dest_location"] = avcs_status_msg.job_info.fms_job_destination_id;

            json_aios_res_refuel_response["data"]["success"] = 1;
            json_aios_res_refuel_response["data"]["rejection_code"] = "";

            resetJobProgressOnNewInstructions();

            // TODO This should be moved to after session verified
            avcs_status_msg.job_progress.ready_for_new_job = false;
        }
        else if(avcs_status_msg.job_progress.ready_for_new_job == true || overide_same_type_job){         
            aios_apm_msgs::JobInfo job_info_temp =  retriveJobInfoFromInstruction(json_avcs_req_refuel_instruction,instruction_job_type,true);
            if(job_info_temp.distance2destination > 0 && job_info_temp.distance2destination < param_acceptable_distance_from_current)
            {
              avcs_status_msg.job_info.fms_job_active_id = job_info_temp.fms_job_id;
              json_response_history["response"].push_back("Destination close - sending arrive");
              job_dest_reached();
              return;
            }
            ROS_INFO_STREAM("Accepting next instruction");
            json_avcs_req_next_job_instruction = aifo_response_msg;
            complete_job_for_new = true;
            next_instruction_job_type = 4;
        }

    }


    /*****************************************************************************************************
    * Handle Job request, 
    * AVCS will send Job instructions
    * Copy Destination into response
    *****************************************************************************************************/
    void AifoHandler::handle_avcs_job_request(nlohmann::json aifo_response_msg)
    { 
        json_avcs_req_job_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];

        nlohmann::json local_previous_path;
        local_previous_path = nlohmann::json::array();
        local_previous_path = json_aios_res_job_response["data"]["current_route"];
        json_aios_res_job_response["data"]["current_route"] = nlohmann::json::array();
        json_aios_res_job_response["data"]["id"] = json_avcs_req_job_instruction["data"]["id"];
        avcs_status_msg.job_info.fms_job_active_id = json_aios_res_job_response["data"]["id"];
        int instruction_job_type = 1;
        next_road_block_instruction_job_type = instruction_job_type;
        json_avcs_road_blocked_next_job_instruction = aifo_response_msg;
        bool overide_same_type_job = avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == instruction_job_type;


        /////////////////////////// Handle Rejections ///////////////////////////////////
        // reject job if not in OP
        if(!check_validity_condition(json_aios_res_job_response, avcs_status_msg.fms_status.operation.operation_mode != 2, "01 - APM not in operation mode")) return;
        // reject job if not ready for new job
        if(!check_validity_condition(json_aios_res_job_response, (!avcs_status_msg.job_progress.ready_for_new_job && !overide_same_type_job), "15 - APM is doing another job")) return;
        // reject job if offloading and has no container mounted
        if(json_avcs_req_job_instruction["data"]["activity"] == 6)
        {
            if(!check_validity_condition(json_aios_res_job_response, avcs_status_msg.job_progress.container_information.mounted_containers.size() == 0, "xx - APM currently doesn't have any container")) return;
        }
       
        // reject job if moving at high speed
        if(apm_preprocess_msg.airs_info.speed_feedback >= 25.0)
        {
            request_job_trigger = true;
            if(!check_validity_condition(json_aios_res_job_response, apm_preprocess_msg.airs_info.speed_feedback >= 25.0, "xx - APM cannot accept job due to speeding.")) return;
        }
       
        
        /////////////////////////// Handle job  ///////////////////////////////////
        if(avcs_status_msg.fms_status.instructions.fms_job_received != true)
        {
            ROS_INFO_STREAM("Handling job");
            json_avcs_req_next_job_instruction = "";          
            next_instruction_job_type = 0;

            /////////////////////////// Update job info ///////////////////////////////////
            avcs_status_msg.job_info = retriveJobInfoFromInstruction(json_avcs_req_job_instruction,instruction_job_type);

            bool near_dest = avcs_status_msg.job_info.distance2destination > 0 && avcs_status_msg.job_info.distance2destination < param_acceptable_distance_from_current;
            if(!check_validity_condition(json_aios_res_job_response, avcs_status_msg.job_info.fms_job_active_id == "", "107 - Couldnt resolve job id.")) return;
            if(!check_validity_condition(json_aios_res_job_response, near_dest , "57 - Destination close - sending arrive"))
            {
              job_dest_reached();
              return;
            }

            /////////////////////////// Update instruction status ///////////////////////////////////
            avcs_status_msg.fms_status.instructions.fms_instruction_job_type = instruction_job_type;
            avcs_status_msg.fms_status.instructions.aios_path_accepted = false;
            avcs_status_msg.fms_status.instructions.fms_responded = false;
            avcs_status_msg.fms_status.instructions.fms_release_nav = false;
            avcs_status_msg.fms_status.instructions.fms_job_received = true;

            /////////////////////////// Update response message ///////////////////////////////////
            json_aios_res_job_response["data"]["dest_location"] = avcs_status_msg.job_info.fms_job_destination_id;

            json_aios_req_dest_arrived["data"]["location"] = json_avcs_req_job_instruction["data"]["next_location"];
            json_aios_req_apm_move["data"]["location"] = json_avcs_req_job_instruction["data"]["next_location"];


            json_aios_res_job_response["data"]["success"] = 1;
            json_aios_res_job_response["data"]["rejection_code"] = "";


            resetJobProgressOnNewInstructions();

            // TODO This should be moved to after session verified
            avcs_status_msg.job_progress.ready_for_new_job = true;
            ROS_INFO_STREAM("Handling job done");
        }
        else if(avcs_status_msg.job_progress.ready_for_new_job == true || overide_same_type_job){       
            aios_apm_msgs::JobInfo job_info_temp =  retriveJobInfoFromInstruction(json_avcs_req_job_instruction,instruction_job_type,true);
            if(job_info_temp.distance2destination > 0 && job_info_temp.distance2destination < param_acceptable_distance_from_current)
            {
              avcs_status_msg.job_info.fms_job_active_id = job_info_temp.fms_job_id;
              json_response_history["response"].push_back("Destination close - sending arrive");
              job_dest_reached();
              return;
            }
            else if( avcs_status_msg.job_progress.inprogress == true && 
                  job_info_temp.fms_job_destination_id == avcs_status_msg.job_info.fms_job_destination_id &&
                  job_info_temp.container_config == avcs_status_msg.job_info.container_config )
            {

              json_aios_res_job_response["data"]["current_route"] =  local_previous_path;
              publish_payload(construct_root_message(json_empty_json, json_aios_res_job_response));;
              json_response_history["response"].push_back("Ignoring same destination job");
              return;
            }

            ROS_INFO_STREAM("Accepting next instruction");
            json_avcs_req_next_job_instruction = aifo_response_msg;
            complete_job_for_new = true;
            next_instruction_job_type = 1;
        }
    }

    /*****************************************************************************************************
    * Handle logon request, 
    * AVCS will return success if all details correct, and APM will switch mode to OP
    * Else AVCS will send not success, and APM will retain in TN mode and send alert to reverify trailer 
    *****************************************************************************************************/
    void AifoHandler::handle_avcs_logon_request(nlohmann::json aifo_response_msg)
    {
        if(!avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in)
            return;

        ROS_INFO_STREAM_COLOUR("green","****************** handle_avcs_logon_request *********************");
        if(aifo_response_msg["response_by_avcs"]["data"]["success"] == 1)
        {
            avcs_status_msg.fms_status.operation.operation_mode = 2;
            avcs_status_msg.fms_status.operation.operation_mode_str = "OP";
            avcs_status_msg.fms_status.connectivity.fms_logged_on = true;
            avcs_status_msg.fms_status.connectivity.fms_trailer_verified = true;  
            avcs_status_msg.fms_status.instructions.fms_instruction_job_type = 0;     
            // viz_avcs_response_msg.data = "SUCCESS - Switched to OP mode";
            // debug_avcs_response_pub_.publish(viz_avcs_response_msg);       
                json_response_history["response"].push_back("SUCCESS - Switched to OP mode");
            job_request();
        }
        else
        {
            ROS_ERROR_STREAM("Logon Unsuccessful - Check Trailer details");
            reset_flags_logon();
        }
        ROS_INFO_STREAM_COLOUR("red",construct_root_message(json_empty_json,json_empty_json));
    }

    /*****************************************************************************************************
    * Handle Cancel Job request
    *****************************************************************************************************/
    void AifoHandler::handle_job_cancel_request(nlohmann::json aifo_response_msg)
    {
        json_avcs_req_cancel_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_cancel_res["data"]["id"] = json_avcs_req_cancel_instruction["data"]["id"];
        
        cancel_job_requested = true;

        avcs_status_msg.fms_status.instructions.fms_instruction_job_type = 0;
        manual_route_instruction_job_type = 0;
        ROS_ERROR_STREAM("****************** Cancel Job Response *********************");
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_cancel_res)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_cancel_res));
        json_response_history["response"].push_back("Cancel Job Response");
    }

    /*****************************************************************************************************
    * Handle Stop Job request
    *****************************************************************************************************/
    void AifoHandler::handle_stop_job_request(nlohmann::json aifo_response_msg)
    {
        json_avcs_req_stop_job_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_stop_job_res["data"]["id"] = json_avcs_req_stop_job_instruction["data"]["id"];

        cancel_job_requested = true;

        transition_request(1);
        sleep(0.1);
        log_off_from_TN_required = true;

        ROS_ERROR_STREAM("****************** Stop Job Response *********************");
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_stop_job_res)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_stop_job_res));
        json_response_history["response"].push_back("Stop Job Response");
    }

    /*****************************************************************************************************
    * Handle Resume Job request
    *****************************************************************************************************/
    void AifoHandler::handle_resume_job_request(nlohmann::json aifo_response_msg)
    {
        json_avcs_req_resume_job_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_resume_job_res["data"]["id"] = json_avcs_req_resume_job_instruction["data"]["id"];

        pause_job = false;

        ROS_ERROR_STREAM("****************** Resume Job Response *********************");
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_resume_job_res)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_resume_job_res));
        json_response_history["response"].push_back("Resume Job Response");
    }

    /*****************************************************************************************************
    * handle_manual_route_request
    *****************************************************************************************************/
    void AifoHandler::handle_manual_route_request(nlohmann::json aifo_response_msg)
    {
        json_avcs_req_manual_route_instruction["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_manual_route_response["data"]["current_route"] = nlohmann::json::array();
        json_aios_res_manual_route_response["data"]["id"] = json_avcs_req_manual_route_instruction["data"]["id"];
        manual_route_responded = false;
        avcs_status_msg.behaviour_request.br_job_resume = false;


        /////////////////////////// Handle Rejections ///////////////////////////////////
        

        /////////////////////////// Handle manual route job  ///////////////////////////////////     
        if(avcs_status_msg.fms_status.instructions.fms_job_received != true)
        {
            ROS_ERROR_STREAM("****************** handle manual_route instruction *********************");
            json_avcs_req_next_job_instruction = "";         
            next_instruction_job_type = 0;
            int instruction_job_type = avcs_status_msg.fms_status.instructions.fms_instruction_job_type;
            json_prev_accepted_path_ll = json_accepted_path_ll;

            /////////////////////////// Update instruction status ///////////////////////////////////
            avcs_status_msg.fms_status.instructions.aios_path_accepted = false;
            avcs_status_msg.fms_status.instructions.fms_responded = false;
            avcs_status_msg.fms_status.instructions.fms_release_nav = false;
            avcs_status_msg.fms_status.instructions.fms_job_received = true;
            manual_route_job_accepted =false;

            if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type < 10)
            {
              instruction_job_type = avcs_status_msg.fms_status.instructions.fms_instruction_job_type + 10;
              avcs_status_msg.fms_status.instructions.fms_instruction_job_type = instruction_job_type;
            }
            
        //     complete_job_for_new = true;
        //     manual_route_job = true;
        //     apm_doing_yard_job = false;

        //     int n = json_avcs_req_manual_route_instruction["data"]["route_dag"].size();
            // std::string destination_label_avcs = json_avcs_req_manual_route_instruction["data"]["route_dag"][n-1]["name"];



             /////////////////////////// Update job info ///////////////////////////////////
            // aios_apm_msgs::JobInfo manual_route_job_info_avcs;
            // manual_route_job_info_avcs= retriveJobInfoFromInstruction(json_avcs_req_manual_route_instruction,10);
            // // ROS_INFO_STREAM_COLOUR("yellow", manual_route_job_info_avcs);
            // avcs_status_msg.job_info.given_route_x = manual_route_job_info_avcs.given_route_x;
            // avcs_status_msg.job_info.given_route_y = manual_route_job_info_avcs.given_route_y;
            // avcs_status_msg.job_info.aios_job_type = manual_route_job_info_avcs.aios_job_type;

            avcs_status_msg.job_info = retriveJobInfoFromInstruction(json_avcs_req_manual_route_instruction,instruction_job_type);

            /////////////////////////// Update response message ///////////////////////////////////
            json_aios_res_manual_route_response["data"]["dest_location"] = avcs_status_msg.job_info.fms_job_destination_id;

            json_aios_req_dest_arrived["data"]["location"] = json_avcs_req_job_instruction["data"]["next_location"];
            json_aios_req_apm_move["data"]["location"] = json_avcs_req_job_instruction["data"]["next_location"];


            json_aios_res_manual_route_response["data"]["success"] = 1;
            json_aios_res_manual_route_response["data"]["rejection_code"] = "";
            json_aios_res_manual_route_response["data"]["dest_location"] = avcs_status_msg.job_info.fms_job_destination_id;
            json_aios_res_manual_route_response["data"]["cntr_size"] = avcs_status_msg.job_info.container_type;

            resetJobProgressOnNewInstructions();
            if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type == 10)
              avcs_status_msg.job_progress.ready_for_new_job = true;
        }
        else {
            ROS_INFO_STREAM("Accepting Manual instruction");
            json_avcs_req_next_job_instruction = aifo_response_msg;
            complete_job_for_new = true;
            next_instruction_job_type = 10;
            
        }
    }

    /*****************************************************************************************************
    * handle_manual_route_confirmation_request
    *****************************************************************************************************/
    void AifoHandler::handle_manual_route_confirmation_request(nlohmann::json aifo_response_msg)
    {
        json_avcs_req_manual_route_confirmation_request["data"] = aifo_response_msg["request_by_avcs"]["data"];
        json_aios_res_manual_route_confirmation_response["data"]["id"] = json_avcs_req_manual_route_confirmation_request["data"]["id"];
        json_aios_res_manual_route_confirmation_response["data"]["dest_location"] = json_avcs_req_manual_route_confirmation_request["data"]["dest_location"];

        ROS_ERROR_STREAM("****************** handle manual_route_confirmation instruction *********************");
        ROS_ERROR_STREAM("Current job: "<<manual_route_instruction_job_type);
        
        if (json_avcs_req_manual_route_confirmation_request["data"]["status"] == 1) {
            manual_route_job_accepted = true;
            avcs_status_msg.fms_status.instructions.fms_release_nav = true;
        }
        else {
            manual_route_job_accepted = false;
            avcs_status_msg.behaviour_request.br_job_resume = true;
        }

      
        ROS_ERROR_STREAM(construct_root_message(json_empty_json,json_aios_res_manual_route_confirmation_response)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,json_aios_res_manual_route_confirmation_response));
        json_response_history["response"].push_back("handle manual_route_confirmation instruction");
    }

    /*****************************************************************************************************
    * handle_armg_instruction_to_v2e
    *****************************************************************************************************/
    void AifoHandler::handle_armg_instruction_to_v2e(nlohmann::json aifo_response_msg)
    {
        ROS_ERROR_STREAM("****************** handle armg instruction *********************");

        // std_msgs::String armg_instruction;
        // armg_instruction.data = aifo_response_msg.dump();
        // armg_instruction_pub_.publish(armg_instruction);

      v2e_info_msg.header.stamp = ros::Time::now();
      v2e_info_msg.timestamp_recv = aifo_response_msg["timestamp"];
      v2e_info_msg.apm_id = aifo_response_msg["apm_id"];
      v2e_info_msg.data_timestamp_armg = aifo_response_msg["request_by_avcs"]["data"]["timestamp"];
      v2e_info_msg.data_valid = aifo_response_msg["request_by_avcs"]["data"]["valid"];
      v2e_info_msg.data_crane_number = aifo_response_msg["request_by_avcs"]["data"]["crane_number"];
      v2e_info_msg.data_dist_remaining = (int)aifo_response_msg["request_by_avcs"]["data"]["dist_remaining"] / 1000.0;
      v2e_info_msg.response_by_avcs = aifo_response_msg["response_by_avcs"].dump();
      std_msgs::Float32 dist_msg;
      dist_msg.data = v2e_info_msg.data_dist_remaining;
      v2e_msg_count++; 



      nlohmann::json v2e_response_json;
      v2e_response_json["type"] = "armg_instruction_response";
      v2e_response_json["data"]["id"] = avcs_status_msg.job_info.fms_job_active_id;
      v2e_response_json["data"]["rejection_code"] = "";

        

      // if(!avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received /*|| !avcs_status_msg.job_progress.docking*/ || apm_preprocess_msg.airs_info.speed_feedback > 5.0 )
      // {
      //   v2e_response_json["data"]["success"] = 0;
      //   v2e_response_json["data"]["action"] = 0;
      //   v2e_response_json["data"]["rejection_code"] = "APM is not ready to accept v2e corrections";
      //   json_response_history["response"].push_back("APM is not ready to accept v2e corrections");
      // }
      // else if(v2e_msg_count > 3)
      // {
      //   v2e_response_json["data"]["success"] = 0;
      //   v2e_response_json["data"]["action"] = 0;
      //   v2e_response_json["data"]["rejection_code"] = "V2E msg count greater than 3";
      //   json_response_history["response"].push_back("V2E - msg count greater than 3");
      // }
      // else 
      if(fabs(v2e_info_msg.data_dist_remaining) < 0.05)
      {
        v2e_response_json["data"]["success"] = 1;
        v2e_response_json["data"]["action"] = 0;
        v2e_dist_pub_.publish(dist_msg);
        json_response_history["response"].push_back("V2E - request to move " + std::to_string(dist_msg.data));
      }
      else if(v2e_info_msg.data_dist_remaining < -3.0 || v2e_info_msg.data_dist_remaining > 5.0)
      {
        v2e_response_json["data"]["success"] = 0;
        v2e_response_json["data"]["action"] = 0;
        v2e_response_json["data"]["rejection_code"] = "Not within range";
        json_response_history["response"].push_back("V2E - Not within range " + std::to_string(dist_msg.data));
      }
      else 
      {
        v2e_response_json["data"]["success"] = 1;
        v2e_response_json["data"]["action"] = 1;
        v2e_dist_pub_.publish(dist_msg);
        json_response_history["response"].push_back("V2E - request to move " + std::to_string(dist_msg.data));
      }

        ROS_ERROR_STREAM(construct_root_message(json_empty_json,v2e_response_json)<<std::endl);
        publish_payload(construct_root_message(json_empty_json,v2e_response_json));
        json_response_history["response"].push_back("armg response sent");
    
    }

    /*****************************************************************************************************
    * handle_lane_speed_limit_request
    *****************************************************************************************************/
    void AifoHandler::handle_lane_speed_limit_request(nlohmann::json aifo_response_msg)
    {
        ROS_ERROR_STREAM("****************** handle lane speed limit request *********************");

        segments_to_reduce_speed.clear();
        updated_lanes_with_reduced_speed_request = true;

        if (aifo_response_msg["segments"].size() > 0) {
            float speed_limit_received = aifo_response_msg["speed_limit"];
            speed_limit_ = std::to_string(speed_limit_received);
            for (auto segment : aifo_response_msg["segments"]) {
                for (auto seg : segment) {
                    segments_to_reduce_speed.push_back(seg["name"]);
                }
            }
        }

    }

    /*****************************************************************************************************
    * handle_lane_block_request
    *****************************************************************************************************/
    void AifoHandler::handle_lane_block_request(nlohmann::json aifo_response_msg)
    {
        ROS_ERROR_STREAM("****************** handle lane block request *********************");

        blocked_segment_names.clear();
        updated_lanes_to_block_request = true;

        if (aifo_response_msg["segments"].size() > 0) {
            for (auto segment : aifo_response_msg["segments"]) {
                for (auto seg : segment) {
                    blocked_segment_names.push_back(seg["name"]);
                }
            }
        }
    }

    /*****************************************************************************************************
    * retrieve_lane_id
    *****************************************************************************************************/
    std::string AifoHandler::retrieve_lane_id(geometry_msgs::Pose2D query_point)
    {
        aide_apm_msgs::GetZoneInfo zone_srv_msg;
        aios_apm_msgs::GetLaneInfo ontology_srv_msg;
        aide_apm_msgs::ZoneInfo block_query_pose_feedback;
        zone_srv_msg.request.query_point = query_point;
        ontology_srv_msg.request.query_point = query_point;
        std::string lane_id = "";
        // if(zone_info_srv.call(zone_srv_msg))
        // {
        //   block_query_pose_feedback = zone_srv_msg.response.zone;
        //   // lane_id = block_query_pose_feedback.current_lane_id;
        // }
        if(ontologyinfo_srv.call(ontology_srv_msg))
        {
          lane_id = ontology_srv_msg.response.info.current_lane_info.lane_id;
        }

        return lane_id;

    } 

    /*****************************************************************************************************
    * retrieve_lane_ids for multiple pose
    *****************************************************************************************************/
    std::string AifoHandler::retrieve_lane_id(nlohmann::json query_json)
    {
        // aide_apm_msgs::GetZoneInfo zone_srv_msg;
        aios_apm_msgs::GetLaneInfo ontology_srv_msg;
        // aide_apm_msgs::ZoneInfo block_query_pose_feedback;
        // zone_srv_msg.request.query_point = query_point;
        ontology_srv_msg.request.query_points_multiple = query_json.dump();
        std::string lane_ids = "";
        // // if(zone_info_srv.call(zone_srv_msg))
        // // {
        // //   block_query_pose_feedback = zone_srv_msg.response.zone;
        // //   // lane_id = block_query_pose_feedback.current_lane_id;
        // // }
        if(ontologyinfo_srv.call(ontology_srv_msg))
        {
          lane_ids = ontology_srv_msg.response.multi_lane_ids;
        }
        else{
          nlohmann::json empty_array_json;
          empty_array_json = nlohmann::json::array();
          lane_ids = empty_array_json.dump();
        }

        return lane_ids;
    }


    /*****************************************************************************************************
    * handle_block_and_speedcontrol_request
    *****************************************************************************************************/
    void AifoHandler::handle_block_and_speedcontrol_request(nlohmann::json map_response_msg)
    {
        ROS_INFO_STREAM_COLOUR("green","****************** handle block and speed restriction *********************");
        json_map_restrictions_data = nlohmann::json();
        bool restart_for_blocks = false;
        road_blocks_xya = aios_apm_msgs::OntologyPath();
        speed_restrict_xya = aios_apm_msgs::OntologyPath();
        nlohmann::json json_map_restrictions_query_data;
        json_map_restrictions_query_data = nlohmann::json::array();
        nlohmann::json json_map_speed_restrictions_query_data;
        json_map_speed_restrictions_query_data = nlohmann::json::array();
        std::vector<int> unique_index;
        std::vector<int> unique_index_speed;
        for (auto map_info : map_response_msg) {   
          ///////////////////////////// Check for road blocks //////////////////////////////     
          if(map_info["action"] == 0)
          {

            for (auto lane_name : map_info["segments"]) {
              geometry_msgs::Pose2D block_query_pose_start;
              geometry_msgs::Pose2D block_query_pose_end;
              std::string road_name_start = lane_name[0]["name"];
              std::string road_name_end = lane_name[1]["name"];
              block_query_pose_start.x =json_avcs_reformatted_map_data[road_name_start]["x"];
              block_query_pose_start.y =json_avcs_reformatted_map_data[road_name_start]["y"];

              // ROS_INFO_STREAM_COLOUR("green",road_name_start);
              // ROS_INFO_STREAM_COLOUR("green",block_query_pose_start);


              block_query_pose_end.x =json_avcs_reformatted_map_data[road_name_end]["x"];
              block_query_pose_end.y =json_avcs_reformatted_map_data[road_name_end]["y"];


              nlohmann::json jsonlane_data;
              jsonlane_data["name"] = road_name_start;
              jsonlane_data["x"] = block_query_pose_start.x;
              jsonlane_data["y"] = block_query_pose_start.y;
              json_map_restrictions_query_data.push_back(jsonlane_data);

              nlohmann::json jsonlane_data_end;
              jsonlane_data_end["name"] = road_name_end;
              jsonlane_data_end["x"] = block_query_pose_end.x;
              jsonlane_data_end["y"] = block_query_pose_end.y;                
              json_map_restrictions_query_data.push_back(jsonlane_data_end);
              

              aios_apm_msgs::OntologyWaypoint road_block_start;
              road_block_start.id = road_name_start;
              road_block_start.pose = block_query_pose_start;
              aios_apm_msgs::OntologyWaypoint road_block_end;
              road_block_end.id = road_name_end;
              road_block_end.pose = block_query_pose_end;
              road_blocks_xya.waypoints.push_back(road_block_start);
              road_blocks_xya.waypoints.push_back(road_block_end);
            }
          }

          
          ///////////////////////////// Check for Speed restrictions //////////////////////////////
          if(map_info["action"] == 1)
          {
            for (auto lane_name : map_info["segments"]) {
              geometry_msgs::Pose2D block_query_pose_start;
              geometry_msgs::Pose2D block_query_pose_end;
              std::string road_name_start = lane_name[0]["name"];
              std::string road_name_end = lane_name[1]["name"];
              block_query_pose_start.x =json_avcs_reformatted_map_data[road_name_start]["x"];
              block_query_pose_start.y =json_avcs_reformatted_map_data[road_name_start]["y"];



              block_query_pose_end.x =json_avcs_reformatted_map_data[road_name_end]["x"];
              block_query_pose_end.y =json_avcs_reformatted_map_data[road_name_end]["y"];


              nlohmann::json jsonlane_data;
              jsonlane_data["name"] = road_name_start;
              jsonlane_data["x"] = block_query_pose_start.x;
              jsonlane_data["y"] = block_query_pose_start.y;
              jsonlane_data["speed"] = map_info["speed_limit"];
              json_map_speed_restrictions_query_data.push_back(jsonlane_data);

              nlohmann::json jsonlane_data_end;
              jsonlane_data_end["name"] = road_name_end;
              jsonlane_data_end["x"] = block_query_pose_end.x;
              jsonlane_data_end["y"] = block_query_pose_end.y;   
              jsonlane_data_end["speed"] = map_info["speed_limit"];             
              json_map_speed_restrictions_query_data.push_back(jsonlane_data_end);



              aios_apm_msgs::OntologyWaypoint road_block_start;
              road_block_start.id = road_name_start;
              road_block_start.pose = block_query_pose_start;
              road_block_start.speed = map_info["speed_limit"];
              aios_apm_msgs::OntologyWaypoint road_block_end;
              road_block_end.id = road_name_end;
              road_block_end.pose = block_query_pose_end;
              road_block_end.speed = map_info["speed_limit"];
              speed_restrict_xya.waypoints.push_back(road_block_start);
              speed_restrict_xya.waypoints.push_back(road_block_end);


              // std::string lane_id = retrieve_lane_id(block_query_pose_start);
              // if(lane_id != "")
              // {
              //   json_map_restrictions_data["speed"].push_back(lane_id);
              //   json_map_restrictions_data["speed"].push_back(map_info["speed_limit"]);
              // }
            }
          }
        }

        nlohmann::json blocked_lanes = nlohmann::json::array();
        blocked_lanes = nlohmann::json::parse(retrieve_lane_id(json_map_restrictions_query_data));
        // std::string multi_lane_id 
        std::string prev_laneid;
        bool unique = false;
        for (int i = 0 ; i<blocked_lanes.size(); i++)
        {    
          std::string lane_id = blocked_lanes[i];
          if(lane_id != "")
          {
            if(prev_laneid != lane_id && i%2==1)
            {
              // unique = true;
              unique_index.push_back(i-1);
              unique_index.push_back(i);
            }
            json_map_restrictions_data["block"].push_back(lane_id);
            // blocked_lane_ids_to_publish.data += " " + lane_id;
            prev_laneid = lane_id;
          } 
        }


        // ROS_INFO_STREAM_COLOUR("green",json_map_speed_restrictions_query_data.dump(2));
        nlohmann::json speed_restrictd_lanes = nlohmann::json::array();
        speed_restrictd_lanes = nlohmann::json::parse(retrieve_lane_id(json_map_speed_restrictions_query_data));

        ROS_INFO_STREAM_COLOUR("green",speed_restrictd_lanes);
        // std::string multi_lane_id 
        std::string prev_speed_laneid;
        for (int i = 0 ; i<speed_restrictd_lanes.size(); i++)
        {    
          std::string lane_id = speed_restrictd_lanes[i];
          if(lane_id != "")
          {
            // ROS_INFO_STREAM("prev: "<<prev_speed_laneid<<",lane: "<< lane_id << ",i: "<<i);
            if(prev_speed_laneid != lane_id && i%2==1)
            {
                // ROS_INFO_STREAM_COLOUR("green",i);
                // ROS_INFO_STREAM_COLOUR("yellow",lane_id);
        //       // unique = true;
              unique_index_speed.push_back(i-1);
              unique_index_speed.push_back(i);
            }
            json_map_restrictions_data["speed"].push_back(lane_id);
            // json_map_restrictions_data["speed"].push_back(map_info["speed_limit"]);
        //     blocked_lane_ids_to_publish.data += " " + lane_id;
            prev_speed_laneid = lane_id;
          } 
        }



        if(restart_for_blocks)
        {
          ROS_INFO_STREAM_COLOUR("green","****************** trajectory blocked*");
          json_avcs_req_next_job_instruction = json_avcs_road_blocked_next_job_instruction;
          complete_job_for_new = true;
          local_job_id = avcs_status_msg.job_info.local_job_id +1;
          ROS_INFO_STREAM("Accepting next instruction " << local_job_id);
          next_instruction_job_type = next_road_block_instruction_job_type;
        }
            
        std::sort(unique_index.rbegin(), unique_index.rend());
        if(param_trim_road_blocks){
          for (int i = 0 ; i<unique_index.size(); i++)
          {    
              road_blocks_xya.waypoints.erase(road_blocks_xya.waypoints.begin() + unique_index.at(i));
          }
        }
        std::sort(unique_index_speed.rbegin(), unique_index_speed.rend());
        if(param_trim_road_blocks){
          for (int i = 0 ; i<unique_index_speed.size(); i++)
          {    
              speed_restrict_xya.waypoints.erase(speed_restrict_xya.waypoints.begin() + unique_index_speed.at(i));
          }
        }


        road_block_xya_pub_.publish(road_blocks_xya);
        speed_restrict_xya_pub_.publish(speed_restrict_xya);

        // ROS_INFO_STREAM_COLOUR("yellow",json_map_restrictions_data);
        // ROS_INFO_STREAM_COLOUR("green",json_map_restrictions_query_data.dump(2));
    }


    /*****************************************************************************************************
    * Once map data is received, save original to file,
    * then convert lat lon to utm and save to file as well
    * then reformat the map data from array format to json with segment name as key
    *****************************************************************************************************/
    void AifoHandler::handle_avcs_map(nlohmann::json map_response_msg)
    {
        json_avcs_converted_map_data  = nlohmann::json();
        json_avcs_reformatted_map_data  = nlohmann::json();
        ROS_INFO_STREAM_COLOUR("green","****************** handle avcs map data *********************");
        ///////////////////////////// Save original map to file //////////////////////////////
        ROS_INFO_STREAM_COLOUR("green","save original map");
        MapCsvFile.open(output_map_csv_file);
        if (MapCsvFile.is_open()) {
          MapCsvFile << map_response_msg.dump() << std::endl;
          MapCsvFile.close();
        }

        ///////////////////////////// Convert lat lon to utm on all map //////////////////////////////
        map_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        map_cloud->header.frame_id = "utm";
        nlohmann::json converted_map_data;
        nlohmann::json formatted_map_data;
        int point_id = 0;
        for (auto map_info : map_response_msg["locations"]) {
            double northing_, easting_;
            gps_common::UTM(map_info["position"][1], map_info["position"][0], &easting_, &northing_);


            nlohmann::json road_coordinates;
            road_coordinates["name"] = map_info["name"];
            road_coordinates["position"][0] = easting_;
            road_coordinates["position"][1] = northing_;
            road_coordinates["rmf_avcs_waypoint_type"] =  map_info["rmf_avcs_waypoint_type"];


            pcl::PointXYZI pi;
            pi.x = easting_;
            pi.y = northing_;
            pi.z = 10;
            pi.intensity = point_id;
            map_cloud ->push_back(pi);
            point_id++;


            // nlohmann::json formatted_coordinates;
            std::string name =map_info["name"];
            json_avcs_reformatted_map_data[name]["x"]=easting_;
            json_avcs_reformatted_map_data[name]["y"]=northing_;
            json_avcs_reformatted_map_data[name]["type"]=map_info["rmf_avcs_waypoint_type"];
            // json_avcs_reformatted_map_data[name]["type"] = (map_info["rmf_avcs_waypoint_type"] == "refueling" && map_info["is_charger"] == true) ? "refueling_spot" : "";
            // json_avcs_reformatted_map_data[name]["type"] = (map_info["rmf_avcs_waypoint_type"] == "maintenance" && map_info["is_deployment_point"] == true) ? "parking_spot" : "";
            // json_avcs_reformatted_map_data[name]["type"] = (map_info["rmf_avcs_waypoint_type"] == "maintenance" && map_info["is_parking_spot"] == true) ? "maintenance_spot" : "";

            // if(map_info["rmf_avcs_waypoint_type"] == "refueling" && map_info["is_charger"] == true)
            // {
            //   ROS_INFO_STREAM_COLOUR("blue",json_avcs_reformatted_map_data[name]["type"] );
            //   json_avcs_reformatted_map_data[name]["type"] = "refueling_spot";
            // }
            // else if(map_info["rmf_avcs_waypoint_type"] == "refueling" && map_info["is_charger"] == false)
            // {
            //   json_avcs_reformatted_map_data[name]["type"] = "";
            // }
            // else 
            if(map_info["rmf_avcs_waypoint_type"] == "maintenance" && map_info["is_deployment_point"] == true)
            {
              json_avcs_reformatted_map_data[name]["type"] = "parking_spot";
            }
            else if(map_info["rmf_avcs_waypoint_type"] == "maintenance" && map_info["is_deployment_point"] == false)
            {
              json_avcs_reformatted_map_data[name]["type"] = "";
            }
            else if(map_info["rmf_avcs_waypoint_type"] == "maintenance" && map_info["is_parking_spot"] == true)
            {
              json_avcs_reformatted_map_data[name]["type"] = "maintenance_spot";
            }
            else if(map_info["rmf_avcs_waypoint_type"] == "maintenance" && map_info["is_parking_spot"] == false)
            {
              json_avcs_reformatted_map_data[name]["type"] = "";
            }




            converted_map_data.push_back(road_coordinates);
        }
        json_avcs_converted_map_data = converted_map_data;
        avcs_map_points_pub_.publish(map_cloud);
        kdtree.setInputCloud(map_cloud);

        ///////////////////////////// Save converted map to file //////////////////////////////
        ROS_INFO_STREAM_COLOUR("green","save converted map");
        ConvertedMapCsvFile.open(output_converted_map_csv_file);
        if (ConvertedMapCsvFile.is_open()) {
          ConvertedMapCsvFile << json_avcs_converted_map_data.dump() << std::endl;
          ConvertedMapCsvFile.close();
        } 

        ReformatedMapCsvFile.open(output_reformat_map_csv_file);
        if (ReformatedMapCsvFile.is_open()) {
          ReformatedMapCsvFile << json_avcs_reformatted_map_data.dump() << std::endl;
          ReformatedMapCsvFile.close();
        }
        map_initialized = true;

    }



    /*****************************************************************************************************
    * Call back function for map related handling.  includes speed restriction and road blocks
    * 1. Parse map data
    * 2. Parse road blocks or speed restrictions
    *****************************************************************************************************/
    void AifoHandler::aifoResponseBLOCKCallback(const std_msgs::String::ConstPtr& msg) {
        std::string str_msg = msg->data;
        nlohmann::json map_response_msg;
        // Try parse incoming message
        try{
            map_response_msg = nlohmann::json::parse(str_msg);
        }
        catch (const std::exception& exc) {
            std::cerr << "\nERROR: Unable to parse data: "<< exc.what() << std::endl;
            return;
        }

        /////////////////////////////// Handle Map Data //////////////////////////////////////
        if(map_response_msg["locations"].size() > 10)
        {
            ROS_INFO_STREAM_COLOUR("yellow","********************** Map data received **************************");
            handle_avcs_map(map_response_msg);
        }

        /////////////////////////// Handle speed restrictions & road blocks ///////////////////////////////////
        if (map_response_msg["blocks"].size() > 0)
        {
            ROS_INFO_STREAM_COLOUR("yellow","********************** Block / Speed restrict received **************************");
            handle_block_and_speedcontrol_request(map_response_msg["blocks"]);
        }

    }


    /*****************************************************************************************************
    * Call back function for all instructions
    * first identify if connected to mqtt broker via the client. then handle each type in individual
    * function
    *****************************************************************************************************/
    void AifoHandler::handleNavigationRequestInstructions(nlohmann::json aifo_response_msg, int requested_instruction_type) {
      if(requested_instruction_type == 1)        
            handle_avcs_job_request(aifo_response_msg);
      else if(requested_instruction_type == 2)
            handle_avcs_maintenance_request(aifo_response_msg);
      else if(requested_instruction_type == 3)
            handle_avcs_parking_request(aifo_response_msg);
      else if(requested_instruction_type == 4)        
            handle_avcs_refuel_request(aifo_response_msg);
      else if(requested_instruction_type >= 10)        
            handle_manual_route_request(aifo_response_msg);


    }
    /*****************************************************************************************************
    * Call back function for all instructions
    * first identify if connected to mqtt broker via the client. then handle each type in individual
    * function
    *****************************************************************************************************/
    void AifoHandler::aifoResponseCallback(const std_msgs::String::ConstPtr& msg) {
        std::string str_msg = msg->data;
        nlohmann::json aifo_response_msg;

        // Try parse incoming message
        try{
            aifo_response_msg = nlohmann::json::parse(str_msg);
        }
        catch (const std::exception& exc) {
            std::cerr << "\nERROR: Unable to parse data: "<< exc.what() << std::endl;

            // viz_avcs_response_msg.data = "Error parsing request";
            // debug_avcs_response_pub_.publish(viz_avcs_response_msg);
                json_response_history["response"].push_back("Error parsing request");
            return;
        }

        // Print incoming data - Only the request / response, not entire message
        if(aifo_response_msg.contains("request_by_avcs"))
          ROS_INFO_STREAM("req: "<<aifo_response_msg["request_by_avcs"]);
        if(aifo_response_msg.contains("response_by_avcs"))
          ROS_INFO_STREAM("res: "<<aifo_response_msg["response_by_avcs"]);
        if(aifo_response_msg.contains("broker_status") && aifo_response_msg["broker_status"] == false)
          ROS_INFO_STREAM("res: "<<aifo_response_msg);


        ///////////////////////// Check Connectivity to client ///////////////////////////////////
        if(!avcs_status_msg.fms_status.connectivity.fms_connected && aifo_response_msg["broker_status"]== true)
        {
            avcs_status_msg.fms_status.connectivity.fms_connected = true;
            fms_connected_time = ros::Time::now();
            fms_connected = true;
            // viz_avcs_response_msg.data = "Connected to client";
            // debug_avcs_response_pub_.publish(viz_avcs_response_msg);
                json_response_history["response"].push_back("Connected to client");
        }
        else if(aifo_response_msg["broker_status"]== false)
        {
            avcs_status_msg.fms_status.connectivity.fms_connected = false;
            fms_connected = false;
            json_response_history["response"].push_back("Client disconnected");
        }


        /////////////////////////// Handle commands from AVCS ///////////////////////////////////
        if(aifo_response_msg["request_by_avcs"]["type"] == "switch_mode")
        {
            json_response_history["response"].push_back("Recieved switch_mode request");
            handle_avcs_op_request(aifo_response_msg);
        }
        else if (aifo_response_msg["response_by_avcs"]["type"] == "logon_response")
        {
            json_response_history["response"].push_back("Recieved logon_response request");
            handle_avcs_logon_request(aifo_response_msg);
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "job_instruction")
        {
            json_response_history["response"].push_back("Recieved job_instruction");
            handleNavigationRequestInstructions(aifo_response_msg,1);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "refuel")
        {
            json_response_history["response"].push_back("Recieved refuel_instruction");
            handleNavigationRequestInstructions(aifo_response_msg,4);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "maintenance")
        {
            json_response_history["response"].push_back("Recieved maintenance_instruction");
            handleNavigationRequestInstructions(aifo_response_msg,2);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "parking")
        {
            json_response_history["response"].push_back("Recieved parking_instruction");
            handleNavigationRequestInstructions(aifo_response_msg,3);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "dock_position")
        {
            json_response_history["response"].push_back("Recieved dock_position request");
            handle_dock_request(aifo_response_msg);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "mount_instruction")
        {
            json_response_history["response"].push_back("Recieved mount_instruction request");
            handle_mount_request(aifo_response_msg);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "offload_instruction")
        {
            json_response_history["response"].push_back("Recieved offload_instruction request");
            handle_offload_request(aifo_response_msg);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "cancel_job")
        {
            json_response_history["response"].push_back("Recieved cancel_job request");
            handle_job_cancel_request(aifo_response_msg);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "stop")
        {
            json_response_history["response"].push_back("Recieved stop request");
            handle_stop_job_request(aifo_response_msg);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "resume")
        {
            json_response_history["response"].push_back("Recieved resume request");
            handle_resume_job_request(aifo_response_msg);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "manual_route")
        {
            json_response_history["response"].push_back("Recieved manual_route request");
            handleNavigationRequestInstructions(aifo_response_msg,10);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "manual_route_confirmation")
        {
            json_response_history["response"].push_back("Recieved manual_route_confirmation request");
            handle_manual_route_confirmation_request(aifo_response_msg);
                
        }
        else if (aifo_response_msg["request_by_avcs"]["type"] == "armg_instruction_request")
        {
            json_response_history["response"].push_back("Recieved armg_instruction_request request");
            handle_armg_instruction_to_v2e(aifo_response_msg);                
        }

        else if (aifo_response_msg["request_by_avcs"]["type"] == "path_update_request")
        {
            json_response_history["response"].push_back("Recieved path_update_request");
            handle_path_update_request(aifo_response_msg);                
        }

        else if (aifo_response_msg["request_by_avcs"]["type"] == "link_up_request")
        {
            json_response_history["response"].push_back("Recieved link_up_request");
            handle_link_up_request(aifo_response_msg);                
        }

        
        // debug_avcs_response_pub_.publish(viz_avcs_response_msg);
        // clear v2e info if received any instructions from AVCS
        v2e_info_msg = aios_apm_msgs::V2eInfo(); 
    }


    //*****************************************************************************************************
    void AifoHandler::actionReqCallback(const aios_apm_msgs::GuiControl::ConstPtr& msg) {

        if (!apm_preprocess_initialized)
            return;

        if(msg->poweron)
            power_on_request();
        if(msg->poweroff)
            power_off_request();
        if(msg->update_info)
            update_trailer_request();
        if(msg->transition)
            transition_request(1);
        if(msg->maintenance)
            transition_request(0);
        // if(msg->login)
        //     login_request();
        if(msg->logout)
            logout_request();
            // log_off_from_TN_required = true;
        if(msg->req_job)
            job_request();
        if(msg->job_dest_reached)
            job_dest_reached();
       
    }




  /************************************************************************************************************
  * Callback to retrieve apm preprocess msg
  ************************************************************************************************************/
    void AifoHandler::aiosAcceptedPathCallback(const aios_apm_msgs::JobPath::ConstPtr& msg) {
      json_accepted_path_ll = nlohmann::json::array();
      pcl::PointCloud<pcl::PointXYZI>::Ptr ll2utm_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      ll2utm_cloud->header.frame_id = "utm";

      if(msg->job_id == avcs_status_msg.job_info.fms_job_id && msg->accepted_traj_ll.size()>0)
      {
        path_received_from_session = true;

          for (size_t i = 0; i < msg->accepted_traj_ll.size(); ++i)
          {
              double lon = (double)msg->accepted_traj_ll[i].x;
              double lat = (double)msg->accepted_traj_ll[i].y;

              nlohmann::json json_pose_msg;
              json_pose_msg["convention"] = 0;
              json_pose_msg["position"][0]= lon;
              json_pose_msg["position"][1]= lat;
              json_pose_msg["heading"] = 0;
              json_accepted_path_ll.push_back(json_pose_msg);

              if(param_viz)
              {
                double northing_, easting_;
                gps_common::UTM(lat, lon,  &easting_, &northing_);
                pcl::PointXYZI p;
                p.x = easting_;
                p.y = northing_;
                p.z = 1;
                p.intensity = 10;
                ll2utm_cloud->push_back(p);
              }
          }

          if(param_viz)
          {
            trajectory_pcl_pub_.publish(ll2utm_cloud);
          }
      }
      
    }



  /************************************************************************************************************
  * Callback to retrieve apm preprocess msg
  ************************************************************************************************************/
    void AifoHandler::checkExceptionHandle() {
        int exception_id  = apm_preprocess_msg.exception_info.exception_status;

        if(exception_id == 0)
            exception_responded = false;

        if(exception_id == 0 || exception_responded || avcs_status_msg.fms_status.operation.operation_mode < 1)
          return;
        mi_count++;
        bool exception_failure_status = (exception_id > 100) ? true : false;
        exception_id_being_handled = exception_id;
        
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << mi_count;
        std::string mi_count_str = ss.str();
        

        std::string date_time = formatDateTime(ros::Time::now(),0,"%Y%m%d%H%M");
        std::string exception_id_avcs = "APM" +(apm_preprocess_msg.platform_info.platform_id) + "_" + date_time + "_" + mi_count_str;
        // boost::uuids::uuid exception_uuid = boost::uuids::random_generator()();
        // json_avcs_req_mi_req["data"]["id"]= (boost::uuids::to_string(exception_uuid));
        json_avcs_req_mi_req["data"]["id"]= exception_id_avcs;
        json_avcs_req_mi_req["data"]["curr_block"]= apm_preprocess_msg.aide_info.yard_id;
        json_avcs_req_mi_req["data"]["curr_slot"]= "0";
        json_avcs_req_mi_req["data"]["curr_lane"]= apm_preprocess_msg.aide_info.current_lane_str;
        json_avcs_req_mi_req["data"]["curr_non_yard_location"]= "unknown";
        json_avcs_req_mi_req["data"]["mi_type_c"]= "1";
        json_avcs_req_mi_req["data"]["mi_console"]= "RE";
        json_avcs_req_mi_req["data"]["console_id"]= "";
        json_avcs_req_mi_req["data"]["mi_state_c"]= "05";
        json_avcs_req_mi_req["data"]["failure_status_i"]= exception_failure_status ? "Y": "N";
        json_avcs_req_mi_req["data"]["failure_1_c"]= "220";
        json_avcs_req_mi_req["data"]["failure_2_c"]= "999";
        json_avcs_req_mi_req["data"]["failure_3_c"]= "999";
        json_avcs_req_mi_req["data"]["failure_4_c"]= "999";
        json_avcs_req_mi_req["data"]["failure_5_c"]= "999";
        json_avcs_req_mi_req["data"]["cntr_n"]= std::to_string(avcs_status_msg.job_progress.container_information.mounted_container_count);
        json_avcs_req_mi_req["data"]["mi_ops_c"]= "0";
        json_avcs_req_mi_req["data"]["mi_ops_p"]= "0";
        json_avcs_req_mi_req["data"]["job_id"]= "";


        ROS_ERROR_STREAM("****************** Exception Trigger *********************");
        ROS_ERROR_STREAM(construct_root_message(json_avcs_req_mi_req,json_empty_json)<<std::endl);
        publish_payload(construct_root_message(json_avcs_req_mi_req,json_empty_json));
        json_response_history["response"].push_back("Exception Trigger");
        exception_responded = true;

    }


  /************************************************************************************************************
  * Callback to retrieve apm preprocess msg
  ************************************************************************************************************/
    void AifoHandler::aiosPreprocessCallback(const aios_apm_msgs::AiosPreprocess::ConstPtr& msg) {
        apm_preprocess_initialized = true;

        apm_preprocess_msg = *msg;
        apm_id = apm_preprocess_msg.platform_info.platform_id;
        if(apm_preprocess_msg.session_info.job_acknowledged)
            avcs_status_msg.job_info.new_job = false;
    }

  /************************************************************************************************************
  * Callback to update apm fuel level. 
  ************************************************************************************************************/
  void AifoHandler::refuelCallback(const std_msgs::Int64::ConstPtr& msg) {
      fuel_level_avcs = msg->data;

  }
  /************************************************************************************************************
  * Callback to display map point info
  ************************************************************************************************************/
  void AifoHandler::rvizSelectedPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {      
      pcl::PointCloud<pcl::PointXYZI>::Ptr msg_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *msg_ptr);
      pcl::PointXYZI selectPoint;
      for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = msg_ptr->begin(); item != msg_ptr->end(); item++)
      {
        if(item->z == 10)
        {
          selectPoint.x = item->x;
          selectPoint.y = item->y;
          selectPoint.z = item->z;
          selectPoint.intensity = item->intensity;
          std_msgs::String point_name;
          point_name.data = json_avcs_converted_map_data[selectPoint.intensity]["name"];
          debug_avcs_map_point_name_pub_.publish(point_name);
          break;
        }
      }

  }


  /************************************************************************************************************
  * Function to check condition to update trailer. 
  * TODO : This function should be merged with the other one which is called by action config
  ************************************************************************************************************/
  void AifoHandler::checkTrailerUpdates() {
    if (avcs_status_msg.fms_status.connectivity.fms_powered_on && !avcs_status_msg.fms_status.connectivity.fms_logged_on){

        if (apm_preprocess_msg.platform_info.trailer_id.compare(avcs_status_msg.fms_status.connectivity.fms_trailer_id) !=0 )
        {
            update_trailer_request();
        }
    }
  }

  /************************************************************************************************************
  * Update trailer details if powered on and on demand
  * will not updat if not powered on or already logged in
  * will also not update if trailer id is invalid
  ************************************************************************************************************/
    bool AifoHandler::update_trailer_request()
    {
        if (!avcs_status_msg.fms_status.connectivity.fms_powered_on || avcs_status_msg.fms_status.connectivity.fms_logged_on){
            ROS_ERROR_STREAM("Cannot update trailer details due to FMS status");
            avcs_status_msg.fms_status.connectivity.fms_trailer_updated = false;
            avcs_status_msg.fms_status.connectivity.fms_trailer_verified = false;
            json_response_history["response"].push_back("Cannot update trailer details due to FMS status");
            return false;
        }
        else if (apm_preprocess_msg.platform_info.trailer_id == "0" || apm_preprocess_msg.platform_info.trailer_id == ""){
            ROS_ERROR_STREAM("Cannot update trailer details due to trailer id invalid");
            avcs_status_msg.fms_status.connectivity.fms_trailer_updated = false;
            avcs_status_msg.fms_status.connectivity.fms_trailer_verified = false;
            json_response_history["response"].push_back("trailer id invalid - Recheck / Re-enter");
            return false;
        }
        else
        {
            avcs_status_msg.fms_status.connectivity.fms_trailer_id = apm_preprocess_msg.platform_info.trailer_id ;
            json_aios_req_update_trailer["data"]["num_trailers"]=1;
            json_aios_req_update_trailer["data"]["trailer_numbers"][0]=apm_preprocess_msg.platform_info.trailer_id + " ";
            json_aios_req_update_trailer["data"]["trailer_seq_numbers"][0]=1;
            ROS_INFO_STREAM_COLOUR("green","****************** Update Trailer *********************");
            ROS_INFO_STREAM_COLOUR("red",construct_root_message(json_aios_req_update_trailer,json_empty_json));
            publish_payload(construct_root_message(json_aios_req_update_trailer,json_empty_json));
            avcs_status_msg.fms_status.connectivity.fms_trailer_updated = true;
            json_response_history["response"].push_back("Updated trailer details to AVCS");
        }
    }




  /************************************************************************************************************
  * Check if connected to fms and not powered on, if not then trigger power on
  * Do not trigger if power off is requested
  ************************************************************************************************************/
  void AifoHandler::establishConnectivityToFMS() {
    if (avcs_status_msg.fms_status.connectivity.fms_connected && !avcs_status_msg.fms_status.connectivity.fms_powered_on && !avcs_status_msg.comms.power_off_requested){
        power_on_request();
    }
  }

  /************************************************************************************************************
  * Set the flags that get reset
  * check if power on is already true ( this could be from past session but still active)
  * if power on should be triggered to avcs, then send payload.
  * Send Poweron request with software version id
  ************************************************************************************************************/
   bool AifoHandler::power_on_request()
    {
        avcs_status_msg.comms.power_off_requested = false;        
        avcs_status_msg.fms_status.operation.operation_mode = 0;
        avcs_status_msg.fms_status.operation.operation_mode_str = "MA";

        if (avcs_status_msg.fms_status.connectivity.fms_powered_on)
            return false;
        avcs_status_msg.comms.power_on_requested = true;
        avcs_status_msg.fms_status.connectivity.fms_powered_on = true;
        json_aios_req_poweron["data"]["software_version"]= param_version;
        ROS_INFO_STREAM_COLOUR("green","****************** POWER ON *********************");
        ROS_ERROR_STREAM(construct_root_message(json_aios_req_poweron,json_empty_json)<<std::endl);
        publish_payload(construct_root_message(json_aios_req_poweron,json_empty_json));
        json_response_history["response"].push_back("Power On requested");
        sleep(1);
    }



  /************************************************************************************************************
  * Check the order of Logoff required
  * if logout from TN, ensure operation mode is 1 before trigger logoff
  * if logout from MA, ensure operation mode is 0 before trigger logoff
  ************************************************************************************************************/
  void AifoHandler::checkLogoffRequest() {
    if (log_off_from_TN_required && avcs_status_msg.fms_status.operation.operation_mode == 1){
        logout_request();
        log_off_from_TN_required = false;
    }
    if (log_off_from_TN_required && avcs_status_msg.fms_status.operation.operation_mode != 1){
        transition_request(1);
    }
    else if (log_off_from_MA_required && avcs_status_msg.fms_status.operation.operation_mode == 0){
        logout_request();
        log_off_from_MA_required = false;
    }
    else if (log_off_from_MA_required && avcs_status_msg.fms_status.operation.operation_mode != 0){
        transition_request(0);
        sleep(0.1);
    }
  }

  /*****************************************************************************************************
  * send logoff request
  * should be still powered on but logged off,
  * Switch to TN mode
  *****************************************************************************************************/
  void AifoHandler::logout_request()
  {      
      reset_flags_logon();
      ROS_INFO_STREAM_COLOUR("green","****************** logout_request *********************");
      ROS_INFO_STREAM_COLOUR("cyan",construct_root_message(json_aios_req_logoff,json_empty_json));
      publish_payload(construct_root_message(json_aios_req_logoff,json_empty_json));
      json_response_history["response"].push_back("logout_request");
  }



  /*****************************************************************************************************
  * Publish current avcs status message
  *****************************************************************************************************/
  void AifoHandler::publishAifoStatus() {
    avcs_status_msg.fms_status.fms_id = 2;
    session_status_pub_.publish(avcs_status_msg);



    debug_fr_container_pub_.publish(fr_contr_pos);
    debug_bk_container_pub_.publish(bk_contr_pos);
    debug_fr_container_id_pub_.publish(fr_contr_id);
    debug_bk_container_id_pub_.publish(bk_contr_id);

    std_msgs::String viz_operation_mode_msg;
    viz_operation_mode_msg.data = avcs_status_msg.fms_status.operation.operation_mode_str;
    debug_avcs_opmode_pub_.publish(viz_operation_mode_msg);

    // blocked_lanes_pub_.publish(blocked_lane_ids_to_publish);

    manual_route_trajectory_pub_.publish(manual_path_trajectory);

    v2e_info_pub_.publish(v2e_info_msg);
  }


  /*****************************************************************************************************
  * Function to send heart beat message to AVCS Server
  *****************************************************************************************************/
  void AifoHandler::sendAVCS_HB_and_res() {
    nlohmann::json json_pose_current_route;
    // int traj_size = apm_preprocess_msg.aidc_info.aidc_trajectory_lat.size();
    // if(avcs_status_msg.job_info.given_route_mandatory)
    //   traj_size = 1;

    bool reject = false;
    std::string reject_reason = "";



    /////////////////////////// Send Heartbeat messages at fixed rate to AVCS (1s) ///////////////////////////////////
    ros::Time current_time = ros::Time::now();
    bool hb_throttle = (current_time - last_hb_time).toSec() > 1;
    if (avcs_status_msg.fms_status.connectivity.fms_powered_on && hb_throttle ){
        if(map_initialized)
        {
            pcl::PointXYZI searchPoint;
            searchPoint.x = apm_preprocess_msg.aide_info.current_pose_gl.x;
            searchPoint.y = apm_preprocess_msg.aide_info.current_pose_gl.y;
            searchPoint.y = apm_preprocess_msg.aide_info.current_pose_gl.y;
            searchPoint.z = 10;

            int K = 1;
            std::vector<int> pointIdxKNNSearch(K);
            std::vector<float> pointKNNSquaredDistance(K);
            if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
            {
              for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
              {
                closest_point_name.data = json_avcs_converted_map_data[(*map_cloud)[ pointIdxKNNSearch[i] ].intensity]["name"];
                // ROS_ERROR_STREAM(closest_point_name.data);
              }
            }
        }
        

        last_hb_time = ros::Time::now();
        publish_payload(construct_root_message(json_empty_json,json_empty_json));
    }


    /////////////////////////// temp ///////////////////////////////////
    if(avcs_status_msg.fms_status.instructions.fms_release_nav)
      return;


    /////////////////////////// retrieve path information once path received, and accept path ///////////////////////////////////
    if(avcs_status_msg.fms_status.instructions.fms_job_received && avcs_status_msg.fms_status.instructions.aios_path_accepted) 
    {     

      if(!timer_active && avcs_status_msg.fms_status.instructions.fms_responded == false && path_received_from_session)
      {
          last_path_accepted_time = ros::Time::now();
          avcs_status_msg.fms_status.instructions.fms_responded = true;
          avcs_status_msg.fms_status.instructions.fms_release_nav = false;
          timer_active = true;
          ROS_INFO_STREAM_COLOUR("green", "Path accepted and response timer triggered and constructing response msg");
          ROS_INFO_STREAM_COLOUR("blue", apm_preprocess_msg.aidc_info.aidc_trajectory_lat.size());
          ROS_INFO_STREAM_COLOUR("blue", apm_preprocess_msg.session_info.fms_job_id);



      }
      else if((!avcs_status_msg.job_info.given_route_mandatory && timer_active && (ros::Time::now() - last_path_accepted_time).toSec()>0.5) || 
             (avcs_status_msg.job_info.given_route_mandatory && manual_route_job_accepted)){
        // avcs_status_msg.fms_status.instructions.fms_responded = true;
        avcs_status_msg.fms_status.instructions.fms_release_nav = true;
        avcs_status_msg.job_progress.inprogress = true;
        timer_active = false;
        ROS_INFO_STREAM_COLOUR("green", "Path accepted and response sent");
        json_response_history["response"].push_back("Path accepted and response sent");
      }
      else if(apm_preprocess_msg.session_info.job_rejected == true){
        ROS_INFO_STREAM_COLOUR("green", "ịn rejeection - path");

      }
      // else {
      //   ROS_WARN_STREAM_THROTTLE(1, "stuck here");

      // }

    }
    else if((apm_preprocess_msg.session_info.session_management_job_status == 21 || 
    apm_preprocess_msg.session_info.session_management_job_status == 93) && 
    !avcs_status_msg.fms_status.instructions.fms_responded)
    {
        avcs_status_msg.fms_status.instructions.fms_responded = true;
        reject = true;
        reject_reason = "path not valid";
    }
    
    
    if(apm_preprocess_msg.session_info.session_management_job_status == 91)
    {
        avcs_status_msg.fms_status.instructions.fms_responded = false;
    }
    else if(apm_preprocess_msg.session_info.session_management_job_status == 92 && 
    !avcs_status_msg.fms_status.instructions.fms_responded && !manual_route_responded)
    {
        avcs_status_msg.fms_status.instructions.fms_responded = true;
    }


    /////////////////////////// Send response to AVCS ///////////////////////////////////
    if(avcs_status_msg.fms_status.instructions.fms_responded == true && 
        avcs_status_msg.fms_status.instructions.fms_release_nav == false)
    {
      avcs_status_msg.fms_status.instructions.fms_responded = false;
      if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type == 1){
        json_aios_res_job_response["data"]["current_route"] = reject ? json_empty_json : json_accepted_path_ll;
        publish_payload(construct_root_message(json_empty_json, json_aios_res_job_response));
      }
      else if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type == 2){
        json_aios_res_maintenance_response["data"]["current_route"] = reject ? json_empty_json : json_accepted_path_ll;
        publish_payload(construct_root_message(json_empty_json, json_aios_res_maintenance_response));
      }
      else if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type == 3){
        json_aios_res_park_response["data"]["current_route"] = reject ? json_empty_json : json_accepted_path_ll;
        publish_payload(construct_root_message(json_empty_json, json_aios_res_park_response));
      }
      else if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type == 4){
        json_aios_res_refuel_response["data"]["current_route"] = reject ? json_empty_json : json_accepted_path_ll;
        publish_payload(construct_root_message(json_empty_json, json_aios_res_refuel_response));
      }
      else if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type >= 10){
        json_aios_res_manual_route_response["data"]["current_route"] = reject ? json_empty_json : json_accepted_path_ll;
        json_aios_res_manual_route_response["data"]["success"] = reject ? 0 : 1;
        json_aios_res_manual_route_response["data"]["rejection_code"] = reject ? reject_reason : "";
        publish_payload(construct_root_message(json_empty_json, json_aios_res_manual_route_response));
        json_response_history["response"].push_back("Reject - " + reject_reason);

        ROS_INFO_STREAM_COLOUR("blue",json_aios_res_manual_route_response.dump());
        manual_route_responded = true;
      }
      // avcs_status_msg.fms_status.instructions.fms_instruction_job_type = 0;
    }

    if (reject){
       avcs_status_msg.job_info.aios_job_type = 0;
      avcs_status_msg.fms_status.instructions.fms_release_nav = true;

    }

  }


  /*****************************************************************************************************
  * Function to retrieve the stored csv of last map
  *****************************************************************************************************/
  void AifoHandler::retrieveLastMap() {    
    map_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    map_cloud->header.frame_id = "utm";
    int point_id = 0;
    for (auto map_info : json_avcs_converted_map_data) {

        pcl::PointXYZI pi;
        pi.x = map_info["position"][0];
        pi.y = map_info["position"][1];
        pi.z = 10;
        pi.intensity = point_id;
        map_cloud ->push_back(pi);
        point_id++;
    }
    avcs_map_points_pub_.publish(map_cloud);

    map_initialized = true;

    kdtree.setInputCloud(map_cloud);
  }


  /************************************************************************************************************
  * Function to save important states to csv for retrieval at startup
  ************************************************************************************************************/
  nlohmann::json AifoHandler::saveToCsvFile() {

    StatusMsgCsvFile.open(output_status_msg_csv_file);
    if (StatusMsgCsvFile.is_open()) {
      nlohmann::json AifoStatus_Json;
      AifoStatus_Json["latest_timestamp"] = ros::Time::now().toSec();
      AifoStatus_Json["fms_status"]["fms_endpoint"] = avcs_status_msg.fms_status.fms_endpoint;
      AifoStatus_Json["fms_status"]["operation"]["operation_mode"] = avcs_status_msg.fms_status.operation.operation_mode;
      AifoStatus_Json["fms_status"]["operation"]["operation_mode_str"] = avcs_status_msg.fms_status.operation.operation_mode_str;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_connected"] = avcs_status_msg.fms_status.connectivity.fms_connected;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_powered_on"] = avcs_status_msg.fms_status.connectivity.fms_powered_on;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_trailer_updated"] = avcs_status_msg.fms_status.connectivity.fms_trailer_updated;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_trailer_verified"] = avcs_status_msg.fms_status.connectivity.fms_trailer_verified;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_trailer_id"] = avcs_status_msg.fms_status.connectivity.fms_trailer_id;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_ready_to_log_in"] = avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_logged_on"] = avcs_status_msg.fms_status.connectivity.fms_logged_on;
      AifoStatus_Json["fms_status"]["connectivity"]["fms_login_session_id"] = avcs_status_msg.fms_status.connectivity.fms_login_session_id;


      AifoStatus_Json["fms_status"]["instructions"]["aios_path_accepted"] = avcs_status_msg.fms_status.instructions.aios_path_accepted;
      AifoStatus_Json["fms_status"]["instructions"]["fms_instruction_job_type"] = avcs_status_msg.fms_status.instructions.fms_instruction_job_type;
      AifoStatus_Json["fms_status"]["instructions"]["fms_job_dock_cmd_received"] = avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received;
      AifoStatus_Json["fms_status"]["instructions"]["fms_job_mount_cmd_received"] = avcs_status_msg.fms_status.instructions.fms_job_mount_cmd_received;
      AifoStatus_Json["fms_status"]["instructions"]["fms_job_offload_cmd_received"] = avcs_status_msg.fms_status.instructions.fms_job_offload_cmd_received;
      AifoStatus_Json["fms_status"]["instructions"]["fms_job_received"] = avcs_status_msg.fms_status.instructions.fms_job_received;
      AifoStatus_Json["fms_status"]["instructions"]["fms_responded"] = avcs_status_msg.fms_status.instructions.fms_responded;
      AifoStatus_Json["fms_status"]["instructions"]["fms_release_nav"] = avcs_status_msg.fms_status.instructions.fms_release_nav;

      AifoStatus_Json["job_info"]["activity"]= avcs_status_msg.job_info.activity;
      AifoStatus_Json["job_info"]["aios_job_location_id"]= avcs_status_msg.job_info.aios_job_location_id;
      AifoStatus_Json["job_info"]["aios_job_type"]= avcs_status_msg.job_info.aios_job_type;
      AifoStatus_Json["job_info"]["container_config"]= avcs_status_msg.job_info.container_config;
      AifoStatus_Json["job_info"]["container_type"]= avcs_status_msg.job_info.container_type;
      AifoStatus_Json["job_info"]["fms_job_destination_id"]= avcs_status_msg.job_info.fms_job_destination_id;
      AifoStatus_Json["job_info"]["fms_job_destination_utm"]["x"]= avcs_status_msg.job_info.fms_job_destination_utm.x;
      AifoStatus_Json["job_info"]["fms_job_destination_utm"]["y"]= avcs_status_msg.job_info.fms_job_destination_utm.y;
      AifoStatus_Json["job_info"]["fms_job_active_id"]= avcs_status_msg.job_info.fms_job_active_id;
      AifoStatus_Json["job_info"]["fms_job_id"]= avcs_status_msg.job_info.fms_job_id;
      AifoStatus_Json["job_info"]["given_route_mandatory"] = avcs_status_msg.job_info.given_route_mandatory;
      

      AifoStatus_Json["job_progress"]["complete"]= avcs_status_msg.job_progress.complete;
      AifoStatus_Json["job_progress"]["current_dock_pos"]= avcs_status_msg.job_progress.current_dock_pos;
      AifoStatus_Json["job_progress"]["docking"] = avcs_status_msg.job_progress.docking;
      AifoStatus_Json["job_progress"]["inprogress"] = avcs_status_msg.job_progress.inprogress;
      AifoStatus_Json["job_progress"]["mounting"] = avcs_status_msg.job_progress.mounting;
      AifoStatus_Json["job_progress"]["offloading"] = avcs_status_msg.job_progress.offloading;
      AifoStatus_Json["job_progress"]["pending"] = avcs_status_msg.job_progress.pending;
      AifoStatus_Json["job_progress"]["reached_dest"] = avcs_status_msg.job_progress.reached_dest;
      AifoStatus_Json["job_progress"]["ready_for_new_job"] = avcs_status_msg.job_progress.ready_for_new_job;

      AifoStatus_Json["job_progress"]["container_information"]["cntr_bk_pos_occupied"] = avcs_status_msg.job_progress.container_information.cntr_bk_pos_occupied;
      AifoStatus_Json["job_progress"]["container_information"]["cntr_fr_pos_occupied"] = avcs_status_msg.job_progress.container_information.cntr_fr_pos_occupied;
      AifoStatus_Json["job_progress"]["container_information"]["mounted_container_count"] = avcs_status_msg.job_progress.container_information.mounted_container_count;
      int container_count = 0;
      for (auto it = avcs_status_msg.job_progress.container_information.mounted_containers.begin(); 
          it != avcs_status_msg.job_progress.container_information.mounted_containers.end(); ++it) 
      {
        AifoStatus_Json["job_progress"]["container_information"]["mounted_containers"][container_count]["id"] = it->container_id; 
        container_count++;
      }     
      int lat_index = 0;
      for (auto it = avcs_status_msg.job_info.given_route_x.begin(); 
          it != avcs_status_msg.job_info.given_route_x.end(); ++it) 
      {
        AifoStatus_Json["job_info"]["given_route_x"][lat_index] = *it; 
        lat_index++;
      }

      int lon_index = 0;
      for (auto it = avcs_status_msg.job_info.given_route_y.begin(); 
          it != avcs_status_msg.job_info.given_route_y.end(); ++it) 
      {
        AifoStatus_Json["job_info"]["given_route_y"][lon_index] = *it; 
        lon_index++;
      }

      StatusMsgCsvFile << AifoStatus_Json.dump() << std::endl;
      StatusMsgCsvFile.close();
    }
    else {
        ROS_ERROR_STREAM("Failed to open the CSV file.");
    }
  }


  /*****************************************************************************************************
  * Function to retrieve the stored csv of last known state and set the variables based on the following
  * conditions:
  * 1. Check if endpoint has changed, if yes do not load last known config
  * 2. if duration since last update is greater than threshold, do not load
  * 3. If last state was not logged in, then do not update
  *          i. trailer details
  *         ii. trailer verified status
  *        iii. logon status
  *         iv. ready for new job
  *****************************************************************************************************/
  void AifoHandler::checkLastKnownOperationStatus() {
    ROS_INFO_STREAM("Checking last known operation status");
    json_response_history["response"].push_back("Checking last known operation status");
    std::string line;

    std::ifstream StatusMsgCsvReadFile (output_status_msg_csv_file);
    if (StatusMsgCsvReadFile.is_open()) {
      nlohmann::json AifoStatus_Json;

      while ( getline (StatusMsgCsvReadFile,line) )
      {
        ROS_ERROR_STREAM(line << '\n');
         AifoStatus_Json = nlohmann::json::parse(line);

        if (avcs_status_msg.fms_status.fms_endpoint.compare(AifoStatus_Json["fms_status"]["fms_endpoint"]) != 0)
          return;

        std_msgs::Float64 latest_msg_time;
        std_msgs::Float64 current_time;
        current_time.data = ros::Time::now().toSec();
        latest_msg_time.data = AifoStatus_Json["latest_timestamp"];

        ROS_INFO_STREAM_COLOUR("cyan",latest_msg_time.data);
        ROS_INFO_STREAM_COLOUR("yellow",current_time.data);
        ROS_INFO_STREAM_COLOUR("red",current_time.data - latest_msg_time.data);
        if(current_time.data - latest_msg_time.data > expiry_duation_of_last_known_message)
          return;

        avcs_status_msg.fms_status.operation.operation_mode = AifoStatus_Json["fms_status"]["operation"]["operation_mode"];
        avcs_status_msg.fms_status.operation.operation_mode_str = AifoStatus_Json["fms_status"]["operation"]["operation_mode_str"];
        avcs_status_msg.fms_status.connectivity.fms_powered_on = AifoStatus_Json["fms_status"]["connectivity"]["fms_powered_on"];
        avcs_status_msg.fms_status.connectivity.fms_trailer_id = AifoStatus_Json["fms_status"]["connectivity"]["fms_trailer_id"];
        avcs_status_msg.fms_status.connectivity.fms_trailer_updated = AifoStatus_Json["fms_status"]["connectivity"]["fms_trailer_updated"];
        avcs_status_msg.fms_status.connectivity.fms_trailer_verified = AifoStatus_Json["fms_status"]["connectivity"]["fms_trailer_verified"];
        avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in = AifoStatus_Json["fms_status"]["connectivity"]["fms_ready_to_log_in"];
        avcs_status_msg.fms_status.connectivity.fms_logged_on = AifoStatus_Json["fms_status"]["connectivity"]["fms_logged_on"];
        avcs_status_msg.fms_status.connectivity.fms_login_session_id = AifoStatus_Json["fms_status"]["connectivity"]["fms_login_session_id"];
        avcs_status_msg.job_progress.ready_for_new_job = AifoStatus_Json["job_progress"]["ready_for_new_job"];


       avcs_status_msg.fms_status.instructions.aios_path_accepted = AifoStatus_Json["fms_status"]["instructions"]["aios_path_accepted"] ;
       avcs_status_msg.fms_status.instructions.fms_instruction_job_type = AifoStatus_Json["fms_status"]["instructions"]["fms_instruction_job_type"] ;
       avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received = AifoStatus_Json["fms_status"]["instructions"]["fms_job_dock_cmd_received"] ;
       avcs_status_msg.fms_status.instructions.fms_job_mount_cmd_received = AifoStatus_Json["fms_status"]["instructions"]["fms_job_mount_cmd_received"] ;
       avcs_status_msg.fms_status.instructions.fms_job_offload_cmd_received = AifoStatus_Json["fms_status"]["instructions"]["fms_job_offload_cmd_received"] ;
       avcs_status_msg.fms_status.instructions.fms_job_received = AifoStatus_Json["fms_status"]["instructions"]["fms_job_received"] ;
       avcs_status_msg.fms_status.instructions.fms_responded = AifoStatus_Json["fms_status"]["instructions"]["fms_responded"] ;
       avcs_status_msg.fms_status.instructions.fms_release_nav = AifoStatus_Json["fms_status"]["instructions"]["fms_release_nav"] ;

       avcs_status_msg.job_info.activity = AifoStatus_Json["job_info"]["activity"];
       avcs_status_msg.job_info.aios_job_location_id = AifoStatus_Json["job_info"]["aios_job_location_id"];
       avcs_status_msg.job_info.aios_job_type = AifoStatus_Json["job_info"]["aios_job_type"];
       avcs_status_msg.job_info.container_config = AifoStatus_Json["job_info"]["container_config"];
       avcs_status_msg.job_info.container_type = AifoStatus_Json["job_info"]["container_type"];
       avcs_status_msg.job_info.fms_job_destination_id = AifoStatus_Json["job_info"]["fms_job_destination_id"];
       avcs_status_msg.job_info.fms_job_destination_utm.x = AifoStatus_Json["job_info"]["fms_job_destination_utm"]["x"];
       avcs_status_msg.job_info.fms_job_destination_utm.y = AifoStatus_Json["job_info"]["fms_job_destination_utm"]["y"];
       avcs_status_msg.job_info.fms_job_active_id = AifoStatus_Json["job_info"]["fms_job_active_id"];
       avcs_status_msg.job_info.fms_job_id = AifoStatus_Json["job_info"]["fms_job_id"];
       avcs_status_msg.job_info.given_route_mandatory = AifoStatus_Json["job_info"]["given_route_mandatory"];

      std::vector<double> lat;

       avcs_status_msg.job_progress.complete = AifoStatus_Json["job_progress"]["complete"];
       avcs_status_msg.job_progress.current_dock_pos = AifoStatus_Json["job_progress"]["current_dock_pos"];
       avcs_status_msg.job_progress.docking = AifoStatus_Json["job_progress"]["docking"] ;
       avcs_status_msg.job_progress.inprogress = AifoStatus_Json["job_progress"]["inprogress"] ;
       avcs_status_msg.job_progress.mounting = AifoStatus_Json["job_progress"]["mounting"] ;
       avcs_status_msg.job_progress.offloading = AifoStatus_Json["job_progress"]["offloading"] ;
       avcs_status_msg.job_progress.pending = AifoStatus_Json["job_progress"]["pending"] ;
       avcs_status_msg.job_progress.reached_dest = AifoStatus_Json["job_progress"]["reached_dest"] ;
       avcs_status_msg.job_progress.ready_for_new_job = AifoStatus_Json["job_progress"]["ready_for_new_job"] ;

       avcs_status_msg.job_progress.container_information.cntr_bk_pos_occupied = AifoStatus_Json["job_progress"]["container_information"]["cntr_bk_pos_occupied"] ;
       avcs_status_msg.job_progress.container_information.cntr_fr_pos_occupied = AifoStatus_Json["job_progress"]["container_information"]["cntr_fr_pos_occupied"] ;
       avcs_status_msg.job_progress.container_information.mounted_container_count = AifoStatus_Json["job_progress"]["container_information"]["mounted_container_count"] ;
     
       for (auto container_id :  AifoStatus_Json["job_progress"]["container_information"]["mounted_containers"]) {
          ROS_ERROR_STREAM(container_id);          
          aios_apm_msgs::ContainerDB container_info;
          container_info.container_id = container_id["id"];
          avcs_status_msg.job_progress.container_information.mounted_containers.push_back(container_info);
        }

       for (auto lat_value :  AifoStatus_Json["job_info"]["given_route_x"]) {
          avcs_status_msg.job_info.given_route_x.push_back(lat_value);
        }

       for (auto lon_value :  AifoStatus_Json["job_info"]["given_route_y"]) {
          avcs_status_msg.job_info.given_route_y.push_back(lon_value);
          avcs_status_msg.job_info.given_route_a.push_back(1.0);
        }

        if(!avcs_status_msg.fms_status.connectivity.fms_logged_on)
        {
          avcs_status_msg.fms_status.connectivity.fms_trailer_id = "";
          avcs_status_msg.fms_status.connectivity.fms_trailer_verified = false;
          avcs_status_msg.fms_status.connectivity.fms_ready_to_log_in = false;
          avcs_status_msg.job_progress.ready_for_new_job = false;
        }
      }
      StatusMsgCsvReadFile.close();
    }
    else {
        ROS_ERROR_STREAM("Failed to open the CSV file.");
    }
  }


  /*****************************************************************************************************
  * Function to check last known Job status
  *****************************************************************************************************/
  void AifoHandler::publishAVCSResponses() {
      while(json_response_history["response"].size()>10)
            json_response_history["response"].erase(json_response_history["response"].begin());

          // ROS_INFO_STREAM_COLOUR("green", json_response_history.dump());
          viz_avcs_response_msg.data = json_response_history["response"].dump(2);
        debug_avcs_response_pub_.publish(viz_avcs_response_msg);
  }

  /*****************************************************************************************************
  * Function to check last known Job status
  *****************************************************************************************************/
  void AifoHandler::checkLastKnownJobStatus() {
    ROS_INFO_STREAM("Checking last known Job status");
  }

  /*****************************************************************************************************
  * Function to check and update avcs job status
  *****************************************************************************************************/
  void AifoHandler::updateAVCSJobStatus() {
    int avcs_job_status = JSON_AVCS_JOB_STATUS["AVCSJS_NOT_IN_OP"];
    std::string avcs_job_status_str = "AVCSJS_NOT_IN_OP";
    int session_job_status = apm_preprocess_msg.session_info.session_management_job_status;
    int activity = avcs_status_msg.job_info.activity;
    if (avcs_status_msg.fms_status.operation.operation_mode == 2)
    {
        avcs_job_status = avcs_status_msg.job_progress.fms_job_status;
        switch(session_job_status){
            case 0:
            case -1:
                avcs_job_status_str = "AVCSJS_WAITING_JOB_INSTRUCTION";
                break;
            case 1:
            case 3:
            case 30:
                avcs_job_status_str = "AVCSJS_PROCESSING_JOB_INSTRUCTION";
                break;
            case 5:
            case 7:
            case 9:
            case 11:
            case 15:
                avcs_job_status_str = "AVCSJS_EXECUTING_JOB_INSTRUCTION";
                break;
            case 13:
                if(activity >1 && activity <9 && activity != 5)
                  avcs_job_status_str = "AVCSJS_WAITING_DOCK_POSITION";
                else if(activity == 5)
                  avcs_job_status_str = "AVCSJS_WAITING_JOB_INSTRUCTION";
                break;
            case 12:
                 if(!avcs_status_msg.job_progress.docking )
                  avcs_job_status_str = "AVCSJS_WAITING_JOB_INSTRUCTION";
                break;
        }   

        if(avcs_status_msg.job_progress.docking)
            avcs_job_status_str = "AVCSJS_EXECUTING_DOCK_POSITION";


       

      
    }

    avcs_status_msg.job_progress.fms_job_status_str = avcs_job_status_str;
    avcs_status_msg.job_progress.fms_job_status = JSON_AVCS_JOB_STATUS[avcs_job_status_str];
    

     
  }

  /*****************************************************************************************************
  * Check current job / Navigation status
  *****************************************************************************************************/
  void AifoHandler::checkJobStatusUpdates() {
    //This should be changed to update once path is accepted from AVCS. by default will be true once reponse is sent
    // will be different if manual route

    avcs_status_msg.fms_status.instructions.aios_path_accepted = apm_preprocess_msg.session_info.next_job_path_computed;
    int session_job_status = apm_preprocess_msg.session_info.session_management_job_status;
    std::string session_active_job_id = apm_preprocess_msg.session_info.fms_job_id;


    /////////////////////////// Reset job stop flag ///////////////////////////////////
    if((session_job_status == -1 || session_job_status == 0) && avcs_status_msg.behaviour_request.br_job_stop)
    {
        avcs_status_msg.behaviour_request.br_job_stop = false;
        avcs_status_msg.job_info =  aios_apm_msgs::JobInfo();
    }

    if(!avcs_status_msg.fms_status.instructions.fms_job_received && !avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received)
      return;

    avcs_status_msg.job_progress.docking = (avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received && session_job_status == 8) ? true : avcs_status_msg.job_progress.docking; 

    int no_of_containers = 0;
    no_of_containers =  avcs_status_msg.job_progress.container_information.mounted_containers.size();
    avcs_status_msg.job_progress.container_information.mounted_container_count = no_of_containers;
    if (no_of_containers == 0)
    {
      avcs_status_msg.job_progress.container_information.cntr_bk_pos_occupied = false;
      avcs_status_msg.job_progress.container_information.cntr_fr_pos_occupied = false;
    }
    bool standby_complete = false;
    bool dock_complete = false;
    bool refuel_complete = false;
    bool maintainenace_complete = false;


    if(complete_job_for_new)
    {
      complete_job_for_new = false;
      standby_complete = true;
    }



    //Determine job completion status once job accepted by session management
    if(session_active_job_id == avcs_status_msg.job_info.fms_job_id)
    {
      if (session_job_status == 13  && !avcs_status_msg.job_progress.reached_dest) 
      {

        if(avcs_status_msg.job_info.activity == 0 && 
          (avcs_status_msg.fms_status.instructions.fms_instruction_job_type == 10 || avcs_status_msg.fms_status.instructions.fms_instruction_job_type == 11))
        {
            job_dest_reached();
            avcs_status_msg.job_progress.inprogress = false;
        }
        
        if(avcs_status_msg.job_info.activity == 1 || avcs_status_msg.job_info.activity == 5 ||
           avcs_status_msg.job_info.activity == 2 || avcs_status_msg.job_info.activity == 6)
        {
            standby_complete = true;
            job_dest_reached();
            if(request_job_trigger)
            {
              request_job_trigger = false;
              job_request();
            }
        }

        if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == 4 && refuel_standby_complete && refuel_handle_kiosk )
        {
            ROS_INFO_STREAM_COLOUR("yellow","reached refual kiosk");
            refuel_complete = true;
            refuel_standby_complete =false;
            refuel_handle_kiosk = false;
            job_dest_reached();
            if(request_job_trigger)
            {
              request_job_trigger = false;
              job_request();
            }
        }
        else if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == 4 && !refuel_standby_complete)
        {
            ROS_INFO_STREAM_COLOUR("yellow","reached refual standby point, prepare to handle to kiosk");
            refuel_standby_complete = true;
            job_dest_reached();
        }

        if(avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == 2 || 
          avcs_status_msg.fms_status.instructions.fms_instruction_job_type % 10 == 3)
        {
            maintainenace_complete = true;
            job_dest_reached();
            avcs_status_msg.job_progress.inprogress = false;
            if(request_job_trigger)
            {
              request_job_trigger = false;
              job_request();
            }
        }
      }
      else if (session_job_status == 12  && avcs_status_msg.job_progress.docking && dock_move_requested)
      {        
        if(avcs_status_msg.job_info.activity == 3 || avcs_status_msg.job_info.activity == 7 || avcs_status_msg.job_info.activity == 4 || avcs_status_msg.job_info.activity == 8)
        {
            dock_complete = true;
            job_dest_reached();
            dock_move_requested = false;
        }
      }

    }


    if(cancel_job_requested)
    {
        ROS_INFO_STREAM_COLOUR("yellow","Cancelling current job");
        cancel_job_requested = false;
        refuel_standby_complete = false;
        avcs_status_msg.behaviour_request.br_job_stop = true;

        avcs_status_msg.job_info =  aios_apm_msgs::JobInfo();
        avcs_status_msg.job_progress.complete = true;
        avcs_status_msg.job_progress.ready_for_new_job = true;
        avcs_status_msg.job_progress.inprogress = false;
        avcs_status_msg.fms_status.instructions.fms_job_received = false;
        avcs_status_msg.fms_status.instructions.aios_path_accepted = false;
        // avcs_status_msg.job_info = aios_apm_msgs::JobInfo();
        v2e_msg_count = 0;
    }
    else if(standby_complete)
    {
        ROS_INFO_STREAM_COLOUR("yellow","Resetting current job for next job");
        avcs_status_msg.job_progress.complete = true;
        avcs_status_msg.job_progress.inprogress = false;
        avcs_status_msg.job_progress.ready_for_new_job = true;
        avcs_status_msg.fms_status.instructions.fms_job_received = false;
        avcs_status_msg.fms_status.instructions.aios_path_accepted = false;
        v2e_msg_count = 0;
    }
    else if(dock_complete)
    {
        ROS_INFO_STREAM_COLOUR("yellow","Dock complete");
        json_response_history["response"].push_back("Dock complete");
        avcs_status_msg.job_progress.complete = true;
        avcs_status_msg.job_progress.ready_for_new_job = false;
        avcs_status_msg.fms_status.instructions.fms_job_dock_cmd_received = false;
        avcs_status_msg.job_progress.docking = false;

        // v2e_msg_count = 0;
    }
    else if(refuel_complete)
    {
        avcs_status_msg.job_progress.complete = true;
        avcs_status_msg.job_progress.ready_for_new_job = true;
        avcs_status_msg.fms_status.instructions.fms_job_received = false;
        avcs_status_msg.fms_status.instructions.aios_path_accepted = false;
        transition_request(1);
        sleep(0.1);
        log_off_from_TN_required = true;
        avcs_status_msg.job_progress.inprogress = false;
        avcs_status_msg.job_progress.reached_dest = true;
        avcs_status_msg.job_info = aios_apm_msgs::JobInfo();
    }
    else if(refuel_standby_complete)
    {
        ROS_INFO_STREAM_THROTTLE(2,"Prepare to handle REFUEL To KIOSK");
        avcs_status_msg.job_progress.complete = true;
        avcs_status_msg.fms_status.instructions.fms_job_received = true;
        avcs_status_msg.job_progress.ready_for_new_job = false;
        avcs_status_msg.fms_status.instructions.aios_path_accepted = false;
        avcs_status_msg.job_info.aios_job_type = 7;
        avcs_status_msg.job_info.given_route_mandatory = false;
        // fix this
        avcs_status_msg.job_progress.reached_dest = false;

        if(session_job_status != 13)
        {
          refuel_handle_kiosk = true;
        }
        
    }
    else if(maintainenace_complete)
    {
        avcs_status_msg.job_progress.complete = true;
        avcs_status_msg.job_progress.ready_for_new_job = true;
        avcs_status_msg.fms_status.instructions.fms_job_received = false;
        avcs_status_msg.fms_status.instructions.aios_path_accepted = false;
        avcs_status_msg.job_progress.inprogress = false;
        transition_request(1);    
        sleep(0.1);
        log_off_from_TN_required = true;
        avcs_status_msg.job_info = aios_apm_msgs::JobInfo();
    } 




      if(next_instruction_job_type > 0)
      {
        ROS_INFO_STREAM_COLOUR("yellow",json_avcs_req_next_job_instruction);
        handleNavigationRequestInstructions(json_avcs_req_next_job_instruction,next_instruction_job_type);
      } 


     


  }








    void AifoHandler::aiosInstructionsCallback(const std_msgs::String::ConstPtr &msg)
    {

      if(!accept_job_via_instructions_)
        return;
        std::string instruction_str_msg = msg->data;
        nlohmann::json instruction_json;



        std::string aios_destination_id;
        std::string fms_destination_id;
        std::string dst_blk_id;
        std::string dst_blk_name;
        std::string dst_lane_id;
        std::string dst_slot_id;
        std::string job_type_prefix;
        int dst_job_type;
        int dst_activity;
        int dst_dock_pos;
        int dst_container_type;
        std::string dst_container_type_str;
        int dst_container_loc;

        // Try parse incoming message
        try{
            instruction_json = nlohmann::json::parse(instruction_str_msg);
        }
        catch (const std::exception& exc) {
            std::cerr << "\nERROR: Unable to parse data: "<< exc.what() << std::endl;
            return;
        }

        
        for (const auto& item : instruction_json["instructions"]) {             
            if(item.contains("type") && item["type"] == "new_destination_request")
            {
                // ROS_ERROR_STREAM(aios_destination_id);
                if(item["payload"].contains("destination_id"))
                {
                    aios_destination_id = item["payload"]["destination_id"];
                }
                if(item["payload"].contains("block_id"))
                {
                    dst_blk_id = std::to_string((int)item["payload"]["block_id"]);
                }
                if(item["payload"].contains("block_name"))
                {
                    dst_blk_name = item["payload"]["block_name"];
                }
                if(item["payload"].contains("lane_id"))
                {
                    dst_lane_id = std::to_string((int)item["payload"]["lane_id"]);
                }
                if(item["payload"].contains("slot_id"))
                {
                    dst_slot_id = std::to_string((int)item["payload"]["slot_id"]);
                }
                if(item["payload"].contains("job_type"))
                {
                    dst_job_type = (int)item["payload"]["job_type"];
                    dst_activity = dst_job_type == 1 ? 2 :
                                    (dst_job_type == 2 ? 6 : 1);


                    job_type_prefix = dst_job_type == 1 ? "MT" :  
                                     (dst_job_type == 2 ? "OL" : 
                                     (dst_job_type == 5 ? "ST" : "UK"));
                }
                if(item["payload"].contains("container_type"))
                {
                    dst_container_type = (int)item["payload"]["container_type"];
                    if(dst_container_type == 4)
                    {
                      dst_container_type_str = "45";
                      dst_container_loc = 5;
                    }
                    else if(dst_container_type == 3)
                    {
                      dst_container_type_str = "40";
                      dst_container_loc = 5;
                    }
                    else if(dst_container_type == 2)
                    {
                      dst_container_type_str = "20";
                      dst_container_loc = 3;
                    }
                    else if(dst_container_type == 1)
                    {
                      dst_container_type_str = "20";
                      dst_container_loc = 1;
                    }
                }

                if(aios_destination_id == "")
                {
                    aios_destination_id = dst_blk_name + dst_blk_id + "_" + (dst_lane_id + "000").substr(0,2) + "_" + dst_slot_id;
                    // ROS_ERROR_STREAM(aios_destination_id);


                    if (dst_blk_name == "MBK" || dst_blk_name == "MB")
                        fms_destination_id = "MB_" + dst_blk_id;
                    else
                        fms_destination_id = dst_blk_name  + dst_blk_id + "_lane_" + dst_lane_id + "_slot_" + dst_slot_id;
                }
                
                // force_override_job = true;
                // force_override_job_destination_id = aios_destination_id;
                // force_override_job_dst_dock_pos = dst_dock_pos;
                // force_override_job_job_type = dst_job_type;
                // force_override_job_dst_container_type = dst_container_type;
                force_override_job_count++;
                // force_override_job_id = "IJO" + std::to_string(force_override_job_count) + "_" + job_type_prefix;
                force_override_job_id = avcs_status_msg.job_info.fms_job_active_id;
                force_override_job_id = force_override_job_id == "" ? ("LJID_" + std::to_string(force_override_job_count)): force_override_job_id;
                force_override_local_job_id = "0";
                // force_override_local_job_id = std::to_string(force_override_job_count);
                // force_override_job_recv_time = ros::Time::now();
                // force_job_timer = nh2.createTimer(ros::Duration(300),&AifoHandler::forceJobTimerCallback, this,true);
                ROS_ERROR_STREAM(aios_destination_id);

                nlohmann::json instruction_based_job;
                nlohmann::json ibj_req;
                nlohmann::json ibj_res;
                nlohmann::json ibj_req_data;
                nlohmann::json route_dag;
                instruction_based_job["timestamp"] =  static_cast<int64_t>(trunc((ros::Time::now().toNSec()/1000000)));

                ibj_res = nlohmann::json();
                ibj_req_data["id"] =  force_override_job_id;
                ibj_req_data["timestamp"] =  ros::Time::now().toSec();
                ibj_req_data["activity"] =  dst_activity;

                route_dag["name"] = fms_destination_id;
                // ibj_req_data["route_dag"] =  route_dag;
                ibj_req_data["assigned_cntr_size"] =  dst_container_type_str;
                ibj_req_data["target_dock_position"] =  dst_container_loc;

                ibj_req_data["route_dag"] =  nlohmann::json::array();
                ibj_req_data["route_dag"].push_back(route_dag);

                // json_pose_msg["convention"] = 0;
                // json_pose_msg["position"][0]= lon;
                // json_pose_msg["position"][1]= lat;
                // json_pose_msg["heading"] = 0;
                // json_accepted_path_ll.push_back(json_pose_msg);

                ibj_req["data"] =  ibj_req_data;
                instruction_based_job["request_by_avcs"] = ibj_req;
                instruction_based_job["response_by_avcs"] =  ibj_res;

                ROS_ERROR_STREAM(instruction_based_job.dump(2));


                handle_avcs_job_request(instruction_based_job);
                // {
                // "timestamp": ros::Time::now().toSec(),
                //  "request_by_avcs": {
                //     "type": "job_instruction", 
                //     "data": {
                //         "id": avcs_status_msg.job_info.fms_job_id ,
                //         "timestamp": ros::Time::now().toSec(), 
                //         "activity": ' +  str(activity) +', 
                //         "assigned_cntr_size": "' + cntr_size + '", 
                //         "target_dock_position": "'+ str(cntr_location) +'", 
                //         }
                //       },
                //       "response_by_avcs": {}
                //       }

            }
        }
    }



   






  /*****************************************************************************************************
  * Destructor
  *****************************************************************************************************/
    AifoHandler::~AifoHandler() {}


/*****************************************************************************************************
*****************************************************************************************************/
int main(int argc, char** argv) {
  ros::init(argc, argv, "aifo_handler_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ROS_INFO_STREAM("Initialising AVCS Handler");
  ros::Rate r(20);

  AifoHandler aifo_handler_obj(nh, private_nh);


  aifo_handler_obj.checkLastKnownOperationStatus();
  aifo_handler_obj.retrieveLastMap();
  aifo_handler_obj.checkLastKnownJobStatus();
  sleep(1);

  while (ros::ok()) {
    if(aifo_handler_obj.apm_id == "")
    {
      ROS_ERROR_STREAM("Preprocess/ APM ID not initialized - retrying in 1 second");
      sleep(1);
    }
    else if(!aifo_handler_obj.fms_connected)
    {
      ROS_ERROR_STREAM("Broker not connected - retrying every 5 second");
      aifo_handler_obj.initialiseMqttComms();
      aifo_handler_obj.publishAVCSResponses();
      sleep(5);
    }
    else{
      aifo_handler_obj.checkLogoffRequest();
      aifo_handler_obj.establishConnectivityToFMS();
      aifo_handler_obj.checkTrailerUpdates();
      aifo_handler_obj.checkJobStatusUpdates();
      aifo_handler_obj.updateAVCSJobStatus();
      aifo_handler_obj.publishAifoStatus();
      aifo_handler_obj.checkExceptionHandle();
      aifo_handler_obj.sendAVCS_HB_and_res();
      aifo_handler_obj.publishAVCSResponses();
      aifo_handler_obj.saveToCsvFile();      
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

