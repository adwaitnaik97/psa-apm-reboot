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
 * author = 'Zeynep Ustun'
 * email  = 'zeynep@aidrivers.ai'
 *
 *******************************************************/

// ROS standard msg Headers

#include <remote_handler/remote_handler.h>

RemoteHandler::RemoteHandler(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh), mqtt_initialized_(false), run_thread_tf_(true), 
     run_thread_ego_(true), emergency_brake_on_(EmergencyBrakeState::INITIAL), 
     isPause_(false), 
     velocity_(0),
     linkup_status_(false),
     is_first_ping_(true)
      
  {
  
  getParams();

  sub_to_aios_preprocess_ =
      private_nh_.subscribe(from_aios_preprocess_topic_, 20, &RemoteHandler::aiosPreprocessCallback,
                            this, ros::TransportHints().tcpNoDelay(true));

  
  sub_to_localization_status_ = private_nh_.subscribe(
     localization_status_topic_, 20, &RemoteHandler::localizationStatusCallback, this,
      ros::TransportHints().tcpNoDelay(true));


  sub_to_vehicle_control_mode_ = private_nh_.subscribe(
      vehicle_control_mode_topic_, 20, &RemoteHandler::vehicleControlModeCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  sub_to_predicted_footprint_ = private_nh_.subscribe(predicted_footprint_topic_, 20,
                                                      &RemoteHandler::predictedFootprintCallback,
                                                      this, ros::TransportHints().tcpNoDelay(true));

  sub_to_route_plan_ = private_nh_.subscribe(route_plan_topic_, 20,
                                             &RemoteHandler::routePlanCallback,
                                             this, ros::TransportHints().tcpNoDelay(true));

  sub_to_ehmi_info_ = private_nh_.subscribe(ehmi_info_topic_, 1, &RemoteHandler::ehmiInfoCallback,
                                            this, ros::TransportHints().tcpNoDelay(true));
  //PCD
  sub_to_vertical_points_ =
      private_nh_.subscribe(vertical_points_topic_, 20, &RemoteHandler::verticalPointsCallback,
                            this, ros::TransportHints().tcpNoDelay(true));

  sub_to_on_road_pc_ =
      private_nh_.subscribe(on_road_pc_topic_, 20, &RemoteHandler::onRoadPCCallback, this,
                            ros::TransportHints().tcpNoDelay(true));

  sub_to_curb_pc_ = private_nh_.subscribe(curb_pc_topic_, 20, &RemoteHandler::curbPCCallback, this,
                                          ros::TransportHints().tcpNoDelay(true));

  sub_to_most_constrained_points_ = private_nh_.subscribe(
      most_constrained_points_topic_, 20, &RemoteHandler::mostConstrainedPointsCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  sub_to_road_intensity_detection_ = private_nh_.subscribe(
      road_intensity_detection_topic_, 20, &RemoteHandler::roadIntensityDetectionCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  //MARKERS

  sub_to_most_constrained_object_ = private_nh_.subscribe(
      most_constrained_object_topic_, 20, &RemoteHandler::mostConstrainedObjectCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  sub_to_objects_topic_ = private_nh_.subscribe(
      objects_topic_, 20, &RemoteHandler::objectsCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  
  sub_to_crosswalk_vis_topic_ = private_nh_.subscribe(
      crosswalk_vis_topic_, 20, &RemoteHandler::crosswalkVisCallback, this,
      ros::TransportHints().tcpNoDelay(true));


  sub_to_traffic_jam_lanes_vis_topic_ = private_nh_.subscribe(
      traffic_jam_lanes_vis_topic_, 20, &RemoteHandler::trafficJamLanesVisCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  sub_to_road_global_vis_topic_  = private_nh_.subscribe(
      road_global_vis_topic_, 20, &RemoteHandler::roadGlobalVisCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  sub_to_objects_of_interest_topic_ = private_nh_.subscribe(
      objects_of_interest_topic_, 20, &RemoteHandler::objectsOfInterestCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  sub_to_hatch_cover_detection_box_topic_ = private_nh_.subscribe(
      hatch_cover_detection_box_topic_, 20, &RemoteHandler::hatchCoverDetectionBoxCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  sub_to_rtg_detection_box_topic_ = private_nh_.subscribe(
      rtg_detection_box_topic_, 20, &RemoteHandler::rtgDetectionBoxCallback, this,
      ros::TransportHints().tcpNoDelay(true));



  sub_to_remote_ops_request_ = private_nh_.subscribe(
      remote_ops_request_topic_, 20, &RemoteHandler::remoteOpsRequestCallback, this,
      ros::TransportHints().tcpNoDelay(true));

  
  sub_to_mqtt_ = nh_.subscribe(from_client_topic_, 20, &RemoteHandler::fromMQTTCallback, this,
                               ros::TransportHints().tcpNoDelay(true));

  /***Publishers***/
  pub_to_mqtt_ = nh_.advertise<std_msgs::String>(to_client_topic_, 1);

  pub_to_aios_instructions_ =
      nh_.advertise<std_msgs::String>(to_aios_instructions_topic_, 1);

  pub_to_ego_state_ =
      nh_.advertise<aios_apm_msgs::EgoState>(to_aios_ego_state_topic_, 20);

  pub_to_vehicle_control_mode_ =
      nh_.advertise<std_msgs::String>(to_aios_vehicle_control_mode_topic_, 20);

  pub_to_tl_state_ =
      nh_.advertise<std_msgs::Int8>(to_aios_tl_state_topic_, 20);
  
  pub_to_linkup_status_ =  nh_.advertise<std_msgs::String>(to_aios_linkup_status_topic_, 1, true);    

  pub_to_remote_ops_init_ = nh_.advertise<std_msgs::String>(to_aios_remote_ops_init_topic_, 1, true);

  pub_to_remote_ops_request_ =  nh_.advertise<std_msgs::String>(remote_ops_request_topic_, 1, true);

  pub_to_job_info_ =  nh_.advertise<std_msgs::String>(to_aios_job_info_topic_, 1, true);


  vec_send_ego_state_.clear();



  job_info_msg_["jobStatus"] = "";
  job_info_msg_["jobDestination"] = "";

  error_reset_timer_ = ros::Timer();
  //Temporarily added to publish remote_ops_request at 2Hz
  remote_ops_request_timer_ = nh_.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&) {
                                              publishRemoteOpsRequest(); });
  

  
}

void RemoteHandler::getParams() {
  
  private_nh_.param<std::string>("from_aios_preprocess_topic", from_aios_preprocess_topic_, "");
  private_nh_.param<std::string>("to_aios_instructions_topic", to_aios_instructions_topic_, "");
  private_nh_.param<std::string>("to_aios_ego_state_topic", to_aios_ego_state_topic_, "");
  private_nh_.param<std::string>("to_aios_tl_state_topic", to_aios_tl_state_topic_, "");
  private_nh_.param<std::string>("to_aios_remote_ops_init_topic", to_aios_remote_ops_init_topic_, "");
  private_nh_.param<std::string>("to_aios_vehicle_control_mode_topic", to_aios_vehicle_control_mode_topic_, "");
  private_nh_.param<std::string>("to_aios_linkup_status_topic", to_aios_linkup_status_topic_, "");
  private_nh_.param<std::string>("to_aios_job_info_topic", to_aios_job_info_topic_, "");
  private_nh_.param<std::string>("localization_status_topic", localization_status_topic_, "");
  private_nh_.param<std::string>("aid_map_frame", aid_map_frame_, "");
  private_nh_.param<std::string>("aid_odom_frame", aid_odom_frame_, "");
  private_nh_.param<std::string>("aid_base_link_frame", aid_base_link_frame_, "");
  private_nh_.param<std::string>("remote_map_frame", remote_map_frame_, "");
  private_nh_.param<std::string>("remote_odom_frame", remote_odom_frame_,"");
  private_nh_.param<std::string>("remote_base_link_frame", remote_base_link_frame_, "");

  ROS_WARN_STREAM("aide_map_frame ["<<aid_map_frame_ <<"] remote_map_frame ["<<remote_map_frame_<<"]");
  ROS_WARN_STREAM("aid_odom_frame ["<<aid_odom_frame_ <<"] remote_odom_frame ["<<remote_odom_frame_<<"]");
  ROS_WARN_STREAM("aid_base_link_frame ["<<aid_base_link_frame_ <<"] remote_base_link_frame ["<<remote_base_link_frame_<<"]");

  private_nh_.param<std::string>("vehicle_control_mode_topic", vehicle_control_mode_topic_, "");
  private_nh_.param<std::string>("predicted_footprint_topic", predicted_footprint_topic_, "");
  private_nh_.param<std::string>("route_plan_topic", route_plan_topic_, "");
  private_nh_.param<std::string>("ehmi_info_topic", ehmi_info_topic_, "");
  private_nh_.param<std::string>("instruction_source", instruction_source_, "");
  //PCD
  private_nh_.param<std::string>("vertical_points_topic", vertical_points_topic_, "");
  private_nh_.param<std::string>("on_road_pc_topic", on_road_pc_topic_, "");
  private_nh_.param<std::string>("curb_pc_topic", curb_pc_topic_, "");
  private_nh_.param<std::string>("most_constrained_points_topic", most_constrained_points_topic_, "");
  private_nh_.param<std::string>("road_intensity_detection_topic", road_intensity_detection_topic_, "");
 
  
  //MARKERS
  private_nh_.param<std::string>("most_constrained_object_topic", most_constrained_object_topic_, "");
  private_nh_.param<std::string>("objects_topic", objects_topic_, "");
  private_nh_.param<std::string>("crosswalk_vis_topic", crosswalk_vis_topic_, "");
  private_nh_.param<std::string>("traffic_jam_lanes_vis_topic", traffic_jam_lanes_vis_topic_, "");
  private_nh_.param<std::string>("road_global_vis_topic", road_global_vis_topic_, "");
  private_nh_.param<std::string>("objects_of_interest_topic", objects_of_interest_topic_, "");
  private_nh_.param<std::string>("hatch_cover_detection_box_topic", hatch_cover_detection_box_topic_, "");
  private_nh_.param<std::string>("rtg_detection_box_topic", rtg_detection_box_topic_, "");

  
  private_nh_.param<std::string>("remote_ops_request_topic", remote_ops_request_topic_, "");
  
  private_nh_.param("ping_frequency", ping_frequency_, 10.0);
  private_nh_.param("ping_timeout", ping_timeout_, 1.0);
  private_nh_.param("main_loop_rate", main_loop_rate_, 20.0);
  private_nh_.param("debug", debug_, false);
  private_nh_.param("enable_breaklink", enable_breaklink_, true);
  private_nh_.param("send_ping_request", send_ping_request_, true);
  private_nh_.param("tf/publish_rate", tf_publish_rate_, 20.0);
  private_nh_.param("ego_state_publish_rate", ego_state_publish_rate_, 20.0);
  
  // private_nh_.param("emergency_brake_duration", emergency_brake_duration_, 1.0);
  XmlRpc::XmlRpcValue tf_frames;
  private_nh_.getParam("tf/frames", tf_frames);
  for (int i = 0; i < tf_frames.size(); i++) 
  {
    nlohmann::json tf_frame;
    tf_frame["frame_id"] = std::string(tf_frames[i]["frame_id"]);
    tf_frame["child_frame_id"]  = std::string(tf_frames[i]["child_frame_id"]);
    ROS_INFO_STREAM(tf_frame.dump());
    tf_frames_.push_back(tf_frame);
  }
  private_nh_.param<std::string>("mqtt/to_client_topic", to_client_topic_, "/");
  private_nh_.param<std::string>("mqtt/from_client_topic", from_client_topic_, "/");

  private_nh_.param<std::string>("mqtt/client_id", client_id_, "");
  private_nh_.param<std::string>("mqtt/pub_namespace", mqtt_pub_topic_namespace_, "");
  private_nh_.param<std::string>("mqtt/sub_namespace", mqtt_sub_topic_namespace_, "");
  private_nh_.param("mqtt/timeout_sec", timeout_sec_, 1.0);
  private_nh_.param<std::string>("mqtt/local_broker_addr", broker_address_, "");

  /************************************************************************************************************
   * Remote Console message files in JSON format
   ************************************************************************************************************/
  parseJsonFile("linkup_json_file");
  parseJsonFile("breaklink_json_file");
  parseJsonFile("teardown_json_file");
  parseJsonFile("ping_json_file");
  parseJsonFile("emergency_brake_command_json_file");
  parseJsonFile("cmd_signal_light_json_file");
  parseJsonFile("cmd_head_light_remote_json_file");
  parseJsonFile("cmd_horn_json_file");
  parseJsonFile("stop_json_file");
  parseJsonFile("traffic_light_override_json_file");
  parseJsonFile("precedence_override_json_file");
  parseJsonFile("adjust_position_json_file");

  parseJsonFile("goal_json_file");  // setDestination
  parseJsonFile("non_yard_goal_json_file");
  parseJsonFile("location_json_file");  // relocalization
  parseJsonFile("manual_push_remote_json_file");
  parseJsonFile("override_path_json_file");

  parseJsonFile("ego_state_json_file");
  
}


uint64_t RemoteHandler::getRosTimeNanosec(const std_msgs::Header& header) {
    // Get the time from the header
    ros::Time stamp = header.stamp;

    // Convert seconds to nanoseconds and add the nanoseconds part
    uint64_t timestamp_ns = static_cast<uint64_t>(stamp.sec) * 1e9 + static_cast<uint64_t>(stamp.nsec);
    
    return timestamp_ns;
}

uint64_t RemoteHandler::getTimestampMillisec() {
  const auto now = std::chrono::system_clock::now();
  // Convert the time point to time since epoch in milliseconds
  auto epoch_time_ms =
      std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
  return epoch_time_ms;  // seconds.count();
}

uint64_t RemoteHandler::getTimestampNanosec() {
  const auto now = std::chrono::system_clock::now();
  // const auto epoch = now.time_since_epoch();

  // Convert the time point to time since epoch in nanoseconds
  auto epoch_time_ns =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();

  //  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
  return epoch_time_ns;  // seconds.count();
}

nlohmann::json RemoteHandler::markerToJson(const visualization_msgs::Marker& marker) {
    
 
    nlohmann::json j;
    try 
    {
      // Convert basic fields
      j["header"]["frame_id"] = marker.header.frame_id;
      j["header"]["stamp"]["secs"] = marker.header.stamp.sec;
      j["header"]["stamp"]["nsecs"] = marker.header.stamp.nsec;
      uint64_t nanosec_ts =
          static_cast<uint64_t>(marker.header.stamp.sec) * 1000000000ULL + marker.header.stamp.nsec;
      j["stamp"] = nanosec_ts;

      j["ns"] = marker.ns;
      j["text"] = marker.text;

      j["id"] = marker.id;
      j["type"] = marker.type;
      j["action"] = marker.action;

      // Convert pose (position + orientation)
      j["pose"]["position"]["x"] = marker.pose.position.x;
      j["pose"]["position"]["y"] = marker.pose.position.y;
      j["pose"]["position"]["z"] = marker.pose.position.z;
      j["pose"]["orientation"]["x"] = marker.pose.orientation.x;
      j["pose"]["orientation"]["y"] = marker.pose.orientation.y;
      j["pose"]["orientation"]["z"] = marker.pose.orientation.z;
      j["pose"]["orientation"]["w"] = marker.pose.orientation.w;

      // Convert scale
      j["scale"]["x"] = marker.scale.x;
      j["scale"]["y"] = marker.scale.y;
      j["scale"]["z"] = marker.scale.z;

      // Convert color
      j["color"]["r"] = marker.color.r;
      j["color"]["g"] = marker.color.g;
      j["color"]["b"] = marker.color.b;
      j["color"]["a"] = marker.color.a;
    
      std::vector<float> points;
      for (size_t i = 0; i < marker.points.size(); i++)
      {
        
          float x = marker.points[i].x;
          float y = marker.points[i].y;
          float z = marker.points[i].z;

          points.push_back(x);
          points.push_back(y);
          points.push_back(z);
    
      }
      
      j["points"] = points;

    }
    catch (const std::exception& exc) {
      std::cerr << "\n1-markerToJson-Exception" << exc.what() << std::endl;
    }
    catch (const nlohmann::json::parse_error& e) 
    {
      std::cerr << "\n2-markerToJson: parse error -"<< e.what() << std::endl;
    }      
    catch (const std::logic_error& e) {
      std::cerr << "\n3-markerToJson- a logic error: " << e.what() << std::endl;
    }
    return j;
}

/*****************************************************************************************************
 * Call back function for all instructions
 * first identify if connected to mqtt broker via the client. then handle each type in individual
 * function
 *****************************************************************************************************/

void RemoteHandler::fromMQTTCallback(const std_msgs::String::ConstPtr& msg) {
 
  std::string str_msg = msg->data;
  nlohmann::json remote_response_msg;
  // Try parse incoming message
  try {
    remote_response_msg = nlohmann::json::parse(str_msg);

  } 
  catch (const std::exception& exc) {
    std::cerr << "\n1-fromMQTTCallback-Exception[std]:" << exc.what() << std::endl;
    return;
  }
  catch (nlohmann::json::type_error& e) {
    std::cerr << "\n2-fromMQTTCallback-Exception[type_error]: " << e.what() << std::endl;
    return;    
  }
  catch (const std::logic_error& e) {
    std::cerr << "\n3-fromMQTTCallback-Exception[logic_error]:" << e.what() << std::endl;
    return;
  }  
  catch (const nlohmann::json::parse_error& e) 
  {
    std::cerr << "\n4-fromMQTTCallback-Exception[parseError]:"<< e.what() << std::endl;
    return;
  }    

  try
  {
    if (remote_response_msg.contains("broker_status")) {
      if (remote_response_msg["broker_status"]) {
        remote_connected_time_ = ros::Time::now();
        broker_connected_ = true;
        ROS_WARN_STREAM_ONCE("MQTT Client connected..");
      } else {
        broker_connected_ = false;
      }
    }
    /********************* Handle requests from Remote Console*********************/
    else {
      std::string topic_name = remote_response_msg["topicName"];
      std::size_t found = topic_name.rfind('/');
      std::string msg_type = "";
      if (found != std::string::npos) {
        msg_type = topic_name.substr(found + 1);
        handleRemoteConsoleRequest(msg_type, remote_response_msg);
      }
    }
  }
  catch (const std::exception& exc) {
    std::cerr << "\n5-fromMQTTCallback-Exception[std]:" << exc.what() << std::endl;
  }
  catch (nlohmann::json::type_error& e) {
    std::cerr << "\n6-fromMQTTCallback-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::logic_error& e) {
    std::cerr << "\n7-fromMQTTCallback-Exception[logic_error]:" << e.what() << std::endl;
  }  
  catch (const nlohmann::json::parse_error& e) 
  {
    std::cerr << "\n8-fromMQTTCallback-Exception[parseError]:"<< e.what() << std::endl;
  }   

}

void RemoteHandler::localizationStatusCallback(const std_msgs::Float32::ConstPtr& msg)
{
  localization_status_ = msg->data;
}

void RemoteHandler::publishAvcsDestinationFromUi(std::string dest)
{
    try
    { 
      ROS_WARN_STREAM("Publishing destination:"<<dest);
      nlohmann::json request_to_mqtt;


      request_to_mqtt["topicName"] = "/remoteops/avcs_destination_from_ui";
      request_to_mqtt["msgType"] = "avcs_destination_from_ui";
      request_to_mqtt["destination"] = dest;
      
      publishToMqtt(request_to_mqtt);

      job_info_msg_["jobDestination"] = dest;
      publishJobInfo();
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-publishAvcsDestinationFromUi-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
      std::cerr<<"\n2-publishAvcsDestinationFromUi-Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
      std::cerr << "\n3-publishAvcsDestinationFromUi-Exception[logic_error]:" << e.what() << std::endl;
    }  
}

void  RemoteHandler::publishAvcsJobStatus(int aios_job_status)
{
      try
      {
        int32_t job_status = static_cast<int32_t>(aios_job_status);
        nlohmann::json request_to_mqtt;
      
        request_to_mqtt["topicName"] = "/remoteops/avcs_job_status";
        request_to_mqtt["msgType"] = "avcs_job_status";
        request_to_mqtt["jobStatus"] = std::to_string(job_status);
      
        publishToMqtt(request_to_mqtt);

        job_info_msg_["jobStatus"] = std::to_string(job_status);
        publishJobInfo();
      }
      catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-publishAvcsJobStatus-Exception[type_error]: " << e.what() << std::endl;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-publishAvcsJobStatus-Exception[std]:" << e.what() << std::endl;
      }  
      catch (const std::logic_error& e) {
        std::cerr << "\n3-publishAvcsJobStatus-Exception[logic_error]:" << e.what() << std::endl;
      }    
}

void  RemoteHandler::publishTrafficLightStatus(int color)
{
    try
    {
     
      uint8_t pub_color;

      if ( color == 0 ) //RED
      {
        pub_color = static_cast<uint8_t>(PsaTrafficLightsStatusEnum::RED);
      }
      else if ( color == 1 )  //AMBER
      {
        pub_color = static_cast<uint8_t>(PsaTrafficLightsStatusEnum::AMBER);
      }
      else if ( color ==  2 ) //GREEN
      {
        pub_color = static_cast<uint8_t>(PsaTrafficLightsStatusEnum::GREEN);
      }
      else if ( color == 3) 
      {
          pub_color = static_cast<uint8_t>(PsaTrafficLightsStatusEnum::GREEN_RIGHT_ARROW);
      }
      else 
      {
        pub_color = static_cast<uint8_t>(PsaTrafficLightsStatusEnum::UNKNOWN);
      }
    

     nlohmann::json request_to_mqtt;
     request_to_mqtt["topicName"] = "/remoteops/traffic_light_status";
     request_to_mqtt["msgType"] = "traffic_light_status";
     request_to_mqtt["color"] = pub_color;

     publishToMqtt(request_to_mqtt);
     
     std_msgs::Int8 pub_msg;

     pub_msg.data = pub_color;

     pub_to_tl_state_.publish(pub_msg); 


    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-publishTrafficLightStatus-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr<<"\n2-publishTrafficLightStatus-Exception[std]:" << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) 
    {
      std::cerr << "\n3-publishTrafficLightStatus-Exception[logic_error]: " << e.what() << std::endl;
    }
}

void RemoteHandler::publishJobInfo()
{  
     std_msgs::String msg;
     msg.data = job_info_msg_.dump();
     pub_to_job_info_.publish(msg);
}

void RemoteHandler::aiosPreprocessCallback(const aios_apm_msgs::AiosPreprocess::ConstPtr& msg) {

  
  aios_apm_msgs::AiosPreprocess apm_preprocess_msg = *msg;
  apm_preprocess_initialized_ = true;
  if (client_id_ == "")
  {
    client_id_ = "teleop" + apm_preprocess_msg.platform_info.platform_id;
  }
  
  if (getLinkupStatus()) 
  {
    velocity_.store(apm_preprocess_msg.airs_info.speed_feedback);
    {
       std::lock_guard<std::mutex> lock(apm_preprocess_mtx_);
       vec_send_ego_state_.push_back(apm_preprocess_msg);
    }
   
    try
    {
        publishTrafficLightStatus( apm_preprocess_msg.aipe_info.aipe_traffic_light_status );
    }
    catch (const std::logic_error& e) {
      std::cerr << "\n1-publishTrafficLightStatus-Exception[logic_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& exc) {
     std::cerr << "\n2-publishTrafficLightStatus-Exception[std]:" << exc.what() << std::endl;
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n3-publishTrafficLightStatus--Exception[type_error]: " << e.what() << std::endl;
    }
   //AVCS Destination
    try
    {
      std::string avcs_dest = apm_preprocess_msg.fleet_management_3rdparty_info.target_loc_id_fms;
      if ((prev_avcs_destination_from_ui_ != avcs_dest) || dest_first_link_up_)
      {
         publishAvcsDestinationFromUi( avcs_dest );
         prev_avcs_destination_from_ui_ = avcs_dest;
         dest_first_link_up_ = false;
      }
    }
    catch (const std::logic_error& e) {
      std::cerr << "\n1-publishAvcsDestinationFromUi-Exception[logic_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& exc) {
       std::cerr << "\n2-publishAvcsDestinationFromUi-Exception[std]: " << exc.what() << std::endl;
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n3-publishAvcsDestinationFromUi-Exception[type_error]: " << e.what() << std::endl;
    } 

    //AVCS job status
    try
    {
      int avcs_job_status = apm_preprocess_msg.fleet_management_3rdparty_info.job_status;
      if (avcs_job_status == -1)
      {
        avcs_job_status = 0;
      }
      if ( (prev_avcs_job_status_ != avcs_job_status) || job_status_first_link_up_)
      {
        publishAvcsJobStatus( avcs_job_status );
        prev_avcs_job_status_ = avcs_job_status;
        job_status_first_link_up_ = false;
      }
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-publishAvcsJobStatus-Exception[type_error]: " << e.what() << std::endl;
    } 
    catch (const std::logic_error& e) {
      std::cerr << "\n2-publishAvcsJobStatus--Exception[logic_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& exc) {
       std::cerr << "\n3-publishAvcsJobStatus-Exception[std]: " << exc.what() << std::endl;
    } 
    
  }
}

void RemoteHandler::remoteOpsRequestCallback(const std_msgs::String::ConstPtr& msg)
{
//  {
//     "alertIds": [0,1],
//     "highRiskDist": [],
//     "highRiskType": [],
//     "jobDoneAllowed": true,
//     "metricValues": ["5", "10"],
//     "restrictedActions": [ ]
// }
 if (getLinkupStatus()) 
 {
    try
    {
      std::string msg_str = msg->data;
      nlohmann::json msg_json = nlohmann::json::parse(msg_str);
      std::vector<uint32_t> vec_alert_id = msg_json["alertIds"].get<std::vector<uint32_t>>();
      std::vector<std::string> vec_values = msg_json["metricValues"].get<std::vector<std::string>>();
      std::vector<uint8_t> vec_restricted_actions = msg_json["restrictedActions"].get<std::vector<uint8_t>>();
      std::vector<uint8_t> vec_high_risk_type = msg_json["highRiskType"].get<std::vector<uint8_t>>();
      std::vector<_Float32> vec_high_risk_dist = msg_json["highRiskDist"].get<std::vector<_Float32>>();
      bool job_done_allowed =  msg_json["jobDoneAllowed"];
      
      nlohmann::json request_to_mqtt;
      request_to_mqtt["topicName"] = "/remoteops/remote_ops_request";
      request_to_mqtt["msgType"] = "remote_ops_request";
      request_to_mqtt["redAlerts"] = nlohmann::json::array();
      request_to_mqtt["redActionableAlerts"] = nlohmann::json::array();
      request_to_mqtt["amberAlerts"] = nlohmann::json::array();
      request_to_mqtt["amberActionableAlerts"] = nlohmann::json::array();
      request_to_mqtt["recommendedActions"] = nlohmann::json::array();
      request_to_mqtt["restrictedActions"] = nlohmann::json::array();
      request_to_mqtt["highRiskType"] = nlohmann::json::array();
      request_to_mqtt["highRiskDist"] = nlohmann::json::array();
      request_to_mqtt["metricTypes"] =  nlohmann::json::array();
      request_to_mqtt["metricValues"] = nlohmann::json::array();
      
      for (size_t i = 0; i < vec_alert_id.size(); i++)
      {
         int alert_id = vec_alert_id[i];
         std::string type = alerts_map_[alert_id]["type"];
         
         if (type == "A") 
         {
           request_to_mqtt["amberAlerts"].push_back(static_cast<uint32_t>(alert_id));
         } 
         else if ( type == "AA" ) 
         {
           request_to_mqtt["amberActionableAlerts"].push_back(static_cast<uint32_t>(alert_id));
         }
         else if ( type == "R" ) 
         {
           request_to_mqtt["redAlerts"].push_back(static_cast<uint32_t>(alert_id));
         }
         else if ( type == "RA" ) 
         {
           request_to_mqtt["redActionableAlerts"].push_back(static_cast<uint32_t>(alert_id));
         }

         if (alert_metric_map_.find(alert_id) != alert_metric_map_.end()) 
         { 
          request_to_mqtt["metricTypes"].push_back(alert_metric_map_[alert_id]["metricID"]);
          if (i < vec_values.size())
          {
            request_to_mqtt["metricValues"].push_back(vec_values[i]);
          }  
         } 
         
        
      }
      

      request_to_mqtt["restrictedActions"] = vec_restricted_actions;
      request_to_mqtt["highRiskType"] = vec_high_risk_type;
      request_to_mqtt["highRiskDist"] = vec_high_risk_dist;
     
     
      request_to_mqtt["jobDoneAllowed"] = job_done_allowed;
     // ROS_WARN_STREAM("Remote ops request:"<<request_to_mqtt.dump()); 
      publishToMqtt(request_to_mqtt);
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-remoteOpsRequestCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr<<"\n2-remoteOpsRequestCallback-Exception[std]:" << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
        std::cerr << "\n3-remoteOpsRequestCallback-Exception[logic_error]:" << e.what() << std::endl;
      }    
 }
}

std::string RemoteHandler::getAiosDesinationtId(uint32_t psaDestinationId)
{
  if ( psaDestinationId == 16 ) //MBstandby13
  {
    return "MBK_XX_13";
  }
  else if  ( psaDestinationId == 17 ) //MBstandby14
  {
    return "MBK_XX_14";   
  }
  else if  ( psaDestinationId == 18 ) //MBstandby15
  {
    return "MBK_XX_15";   
  }
  else if  ( psaDestinationId == 39 ) //MBstandby18
  {
    return "MBK_XX_18";   
  }
  else if  ( psaDestinationId == 40 ) //MBstandby16
  {
    return "MBK_XX_16";   
  }
  else if  ( psaDestinationId == 41 ) //MBstandby17
  {
    return "MBK_XX_17";   
  }
  else if  ( psaDestinationId == 42 ) //Refuel42
  {
    return "RF_XX_4_2";   
  }
  else if  ( psaDestinationId == 43 ) //Refuel32
  {
    return "RF_XX_3_2";   
  }
  else if  ( psaDestinationId == 44 ) //Refuel12
  {
    return "RF_XX_1_2";   
  }
  else if  ( psaDestinationId == 45 ) //Refuel22
  {
    return "RF_XX_2_2";   
  }
  else if  ( psaDestinationId == 46 ) //Refuel21
  {
    return "RF_XX_2_1";   
  }
  else if  ( psaDestinationId == 47 ) //Refuel41
  {
    return "RF_XX_4_1";   
  }
  else if  ( psaDestinationId == 48 ) //Refuel31
  {
    return "RF_XX_3_1";   
  }
  else if  ( psaDestinationId == 49 ) //Refuel11
  {
    return "RF_XX_1_1";   
  }
  else if (psaDestinationId >= 51 && psaDestinationId <= 64) //different slots in RS68
  {
    int d = (psaDestinationId - 51) +1;
    std::string aios_id = "RS_XX_" + std::to_string(d);
    return aios_id; 

  }

  return "";

}; 
//Marker
void RemoteHandler::mostConstrainedObjectCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
 if (getLinkupStatus()) 
 {
   try 
   {
    nlohmann::json request_to_mqtt;
    visualization_msgs::Marker marker_msg = *msg;
    
    request_to_mqtt["topicName"] = "/remoteops/most_constrained_object";
    request_to_mqtt["msgType"] = "most_constrained_object";
    nlohmann::json j = markerToJson(marker_msg);
    request_to_mqtt["marker"] = j;
    
    publishToMqtt(request_to_mqtt);
   } 
   catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-mostConstrainedObjectCallback-Exception[type_error]: " << e.what() << std::endl;
   }
   catch (const std::exception& e) {
        std::cerr<<"\n2-mostConstrainedObjectCallback-Exception[std]: " << e.what() << std::endl;
   }  
   catch (const std::logic_error& e) {
        std::cerr << "\n3-mostConstrainedObjectCallback-Exception[logic_error]:" << e.what() << std::endl;
   }    
 } 
}

void RemoteHandler::objectsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
 if (getLinkupStatus()) 
 {
   try 
   {
    nlohmann::json request_to_mqtt;
    visualization_msgs::MarkerArray marker_array = *msg;
    
    request_to_mqtt["topicName"] = "/remoteops/objects";
    request_to_mqtt["msgType"] = "objects";
    std::vector<nlohmann::json> vec_marker_array;
    for (size_t i = 0; i<marker_array.markers.size(); i++)
    {
        auto j = markerToJson(marker_array.markers[i]);
    
        vec_marker_array.push_back(j);
    } 
    request_to_mqtt["markers"] = vec_marker_array;
    
    publishToMqtt(request_to_mqtt);
   } 
   catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-objectsCallback-Exception[type_error]: " << e.what() << std::endl;
   }
   catch (const std::exception& e) {
        std::cerr<<"\n2-objectsCallback-Exception[std]: " << e.what() << std::endl;
   }  
   catch (const std::logic_error& e) {
        std::cerr << "\n3-objectsCallback-Exception[logic_error]:" << e.what() << std::endl;
   }    
 } 
}

void RemoteHandler::crosswalkVisCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  if (getLinkupStatus()) 
  {
    try 
    {
      nlohmann::json request_to_mqtt;
      visualization_msgs::MarkerArray marker_array = *msg;
      
      request_to_mqtt["topicName"] = "/remoteops/crosswalk_vis";
      request_to_mqtt["msgType"] = "crosswalk_vis";
      std::vector<nlohmann::json> vec_marker_array;
      for (size_t i = 0; i<marker_array.markers.size(); i++)
      {
          auto j = markerToJson(marker_array.markers[i]);
      
          vec_marker_array.push_back(j);
      } 
      request_to_mqtt["markers"] = vec_marker_array;
      
      publishToMqtt(request_to_mqtt);
    } 
    catch (nlohmann::json::type_error& e) {
          std::cerr << "\n1-crosswalkVisCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
          std::cerr<<"\n2-crosswalkVisCallback-Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
          std::cerr << "\n3-crosswalkVisCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  } 

}

void RemoteHandler::trafficJamLanesVisCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  if (getLinkupStatus()) 
  {
    try 
    {
      nlohmann::json request_to_mqtt;
      visualization_msgs::MarkerArray marker_array = *msg;
      
      request_to_mqtt["topicName"] = "/remoteops/traffic_jam_lanes_vis";
      request_to_mqtt["msgType"] = "traffic_jam_lanes_vis";
      std::vector<nlohmann::json> vec_marker_array;
      for (size_t i = 0; i<marker_array.markers.size(); i++)
      {
          auto j = markerToJson(marker_array.markers[i]);
      
          vec_marker_array.push_back(j);
      } 
      request_to_mqtt["markers"] = vec_marker_array;
      
      publishToMqtt(request_to_mqtt);
    } 
    catch (nlohmann::json::type_error& e) {
          std::cerr << "\n1-trafficJamLanesVisCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
          std::cerr<<"\n2-trafficJamLanesVisCallback-Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
          std::cerr << "\n3-trafficJamLanesVisCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  } 

}

void RemoteHandler::roadGlobalVisCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  if (getLinkupStatus()) 
  {
    try 
    {
      nlohmann::json request_to_mqtt;
      visualization_msgs::MarkerArray marker_array = *msg;
      
      request_to_mqtt["topicName"] = "/remoteops/road_global_vis";
      request_to_mqtt["msgType"] = "road_global_vis";
      std::vector<nlohmann::json> vec_marker_array;
      for (size_t i = 0; i<marker_array.markers.size(); i++)
      {
          auto j = markerToJson(marker_array.markers[i]);
      
          vec_marker_array.push_back(j);
      } 
      request_to_mqtt["markers"] = vec_marker_array;
      
      publishToMqtt(request_to_mqtt);
    } 
    catch (nlohmann::json::type_error& e) {
          std::cerr << "\n1-roadGlobalVisCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
          std::cerr<<"\n2-roadGlobalVisCallback-Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
          std::cerr << "\n3-roadGlobalVisCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  } 
}
void RemoteHandler::objectsOfInterestCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  if (getLinkupStatus()) 
  {
    try 
    {
      nlohmann::json request_to_mqtt;
      visualization_msgs::MarkerArray marker_array = *msg;
      
      request_to_mqtt["topicName"] = "/remoteops/objects_of_interest";
      request_to_mqtt["msgType"] = "objects_of_interest";
      std::vector<nlohmann::json> vec_marker_array;
      for (size_t i = 0; i<marker_array.markers.size(); i++)
      {
          auto j = markerToJson(marker_array.markers[i]);
      
          vec_marker_array.push_back(j);
      } 
      request_to_mqtt["markers"] = vec_marker_array;
      
      publishToMqtt(request_to_mqtt);
    } 
    catch (nlohmann::json::type_error& e) {
          std::cerr << "\n1-objectsOfInterestCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
          std::cerr<<"\n2-objectsOfInterestCallback-Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
          std::cerr << "\n3-objectsOfInterestCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  } 

}
void RemoteHandler::hatchCoverDetectionBoxCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  if (getLinkupStatus()) 
  {
    try 
    {
      nlohmann::json request_to_mqtt;
      visualization_msgs::MarkerArray marker_array = *msg;
      
      request_to_mqtt["topicName"] = "/remoteops/hatch_cover_detection_box";
      request_to_mqtt["msgType"] = "hatch_cover_detection_box";
      std::vector<nlohmann::json> vec_marker_array;
      for (size_t i = 0; i<marker_array.markers.size(); i++)
      {
          auto j = markerToJson(marker_array.markers[i]);
      
          vec_marker_array.push_back(j);
      } 
      request_to_mqtt["markers"] = vec_marker_array;
      
      publishToMqtt(request_to_mqtt);
    } 
    catch (nlohmann::json::type_error& e) {
          std::cerr << "\n1-hatchCoverDetectionBoxCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
          std::cerr<<"\n2-hatchCoverDetectionBoxCallback-Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
          std::cerr << "\n3-hatchCoverDetectionBoxCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  } 

}
void RemoteHandler::rtgDetectionBoxCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  if (getLinkupStatus()) 
  {
    try 
    {
      nlohmann::json request_to_mqtt;
      visualization_msgs::MarkerArray marker_array = *msg;
      
      request_to_mqtt["topicName"] = "/remoteops/rtg_detection_box";
      request_to_mqtt["msgType"] = "rtg_detection_box";
      std::vector<nlohmann::json> vec_marker_array;
      for (size_t i = 0; i<marker_array.markers.size(); i++)
      {
          auto j = markerToJson(marker_array.markers[i]);
      
          vec_marker_array.push_back(j);
      } 
      request_to_mqtt["markers"] = vec_marker_array;
      
      publishToMqtt(request_to_mqtt);
    } 
    catch (nlohmann::json::type_error& e) {
          std::cerr << "\n1-rtgDetectionBoxCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
          std::cerr<<"\n2-rtgDetectionBoxCallback-Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
          std::cerr << "\n3-rtgDetectionBoxCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  } 
}


void RemoteHandler::routePlanCallback(const geometry_msgs::Polygon::ConstPtr& msg)
{
   if (getLinkupStatus()) {
    
    try
    {
      nlohmann::json request_to_mqtt;
      std::vector<float> points_x;
      std::vector<float> points_y;
      std::vector<float> points_yaw;

      request_to_mqtt["topicName"] = "/remoteops/route_plan";
      request_to_mqtt["msgType"] = "route_plan";

      request_to_mqtt["timestamp"] = getTimestampNanosec();

      for (size_t i = 0; i < msg->points.size(); i++) {
        points_x.push_back(msg->points[i].x);
        points_y.push_back(msg->points[i].y);
        points_yaw.push_back(msg->points[i].z);
      }

      request_to_mqtt["segmentId"] = nlohmann::json::array();
      request_to_mqtt["discretizedPathPointsX"] = points_x;
      request_to_mqtt["discretizedPathPointsY"] = points_y;
      request_to_mqtt["discretizedPathPointsYaw"] = points_yaw;
      ROS_WARN_STREAM_ONCE("Publishing ROUTE PLAN:"<<request_to_mqtt.dump());

    
      publishToMqtt(request_to_mqtt);
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-routePlanCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr<<"\n2-routePlanCallback-Exception[std]: " << e.what() << std::endl;
    } 
    catch (const std::logic_error& e) {
        std::cerr << "\n3-routePlanCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  }

}

void RemoteHandler::predictedFootprintCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
  if (getLinkupStatus()) {
    
    try
    {
      nlohmann::json request_to_mqtt;
      std::vector<float> points;

      request_to_mqtt["topicName"] = "/remoteops/predicted_footprint";
      request_to_mqtt["msgType"] = "predicted_footprint";

      request_to_mqtt["stamp"] = getTimestampNanosec();

      for (size_t i = 0; i < msg->points.size(); i++) {
        points.push_back(msg->points[i].x);
        points.push_back(msg->points[i].y);
        //points.push_back(msg->points[i].z);
      }

      request_to_mqtt["includesZ"] = false;
      request_to_mqtt["payload"] = points;


      publishToMqtt(request_to_mqtt);
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-predictedFootprintCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr<<"\n2-predictedFootprintCallback-Exception[std]: " << e.what() << std::endl;
    } 
    catch (const std::logic_error& e) {
        std::cerr << "\n3-predictedFootprintCallback-Exception[logic_error]:" << e.what() << std::endl;
    }    
  }
}

void RemoteHandler::vehicleControlModeCallback(const std_msgs::Int16::ConstPtr& msg) {
  if (getLinkupStatus()) {
   try
   {
      int msg_data = msg->data;
      uint8_t mode;
      std::string mode_str;
      
      if (msg_data == 1 ) //Manual
      {
        mode = static_cast<uint8_t>(PsaVehicleControlModeEnum::MANUAL);
        mode_str = "MANUAL";
      }
      else if (msg_data == 2 ) //Auto
      {
        mode = static_cast<uint8_t>(PsaVehicleControlModeEnum::AUTO);
        mode_str = "AUTO";
      }
      else if (msg_data == 3 ) //INTERVENTION
      {
        mode = static_cast<uint8_t>(PsaVehicleControlModeEnum::INTERVENTION);
        mode_str = "INTERVENTION";
      }
      else if (msg_data == 4 ) //EMERGENCY
      {
        mode = static_cast<uint8_t>(PsaVehicleControlModeEnum::EMERGENCY);
        mode_str = "EMERGENCY";
      }
      else if (msg_data == 5 ) //PAUSE
      {
        mode = static_cast<uint8_t>(PsaVehicleControlModeEnum::PAUSE);
        mode_str = "PAUSE";
      }
      else 
      {
       return;
      }

           
      nlohmann::json request_to_mqtt;

      request_to_mqtt["topicName"] = "/remoteops/vehicle_control_mode";
      request_to_mqtt["msgType"] = "vehicle_control_mode";
      request_to_mqtt["mode"] = mode;
      request_to_mqtt["pauseResume"] = true;
          
      
      if ( isPause_.load() ) //Pause
      {
        if (velocity_.load() == 0) 
        {
          request_to_mqtt["pauseResume"] = false;
        }
          
     }
      
     publishToMqtt(request_to_mqtt);

     std_msgs::String pub_vcm_msg;
     nlohmann::json vcm_json;
     vcm_json["mode"] = mode_str;
     vcm_json["pauseFlag"] = request_to_mqtt["pauseResume"];
     pub_vcm_msg.data = vcm_json.dump();
     pub_to_vehicle_control_mode_.publish(pub_vcm_msg);
     
   }
   catch (nlohmann::json::type_error& e) {
        std::cerr << "1-vehicleControlModeCallback-Exception[type_error]: " << e.what() << std::endl;
   }
   catch (const std::exception& e) {
        std::cerr<<"2-vehicleControlModeCallback-Exception[std]: " << e.what() << std::endl;
   } 
   catch (const std::logic_error& e) {
        std::cerr << "\n3-vehicleControlModeCallback-Exception[logic_error]:" << e.what() << std::endl;
   } 
   catch (const nlohmann::json::parse_error& e) 
   {
        std::cerr << "\n4-vehicleControlModeCallback-Exception[parse_error]:"<< e.what() << std::endl;
   }     
  }
}

void RemoteHandler::verticalPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (getLinkupStatus()) {
    try
    {
      sensor_msgs::PointCloud point_cloud;
      // Convert the PointCloud2 to PointCloud
      if (!sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud)) {
        ROS_ERROR("[verticalPointsCallback]-Failed to convert PointCloud2 to PointCloud.");
        return;
      }

      nlohmann::json request_to_mqtt;
      std::vector<float> points;

      request_to_mqtt["topicName"] = "/remoteops/vertical_points";
      request_to_mqtt["msgType"] = "vertical_points";

      uint64_t nanosec_ts =
          static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nsec;

      request_to_mqtt["stamp"] = nanosec_ts;

      for (size_t i = 0; i < point_cloud.points.size(); i++) {
        points.push_back(point_cloud.points[i].x);
        points.push_back(point_cloud.points[i].y);
        points.push_back(point_cloud.points[i].z);
      }

      request_to_mqtt["includesZ"] = true;
      request_to_mqtt["includesIntensity"] = false;
      request_to_mqtt["payload"] = points;

      publishToMqtt(request_to_mqtt);
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "1-verticalPointsCallback-Exception: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr<<"2-verticalPointsCallback: Exception: " << e.what() << std::endl;
    }
  }
}

void RemoteHandler::onRoadPCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (getLinkupStatus()) {
    try
    {
      sensor_msgs::PointCloud point_cloud;
      // Convert the PointCloud2 to PointCloud
      if (!sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud)) {
        ROS_ERROR("Failed to convert PointCloud2 to PointCloud.");
        return;
      }

      nlohmann::json request_to_mqtt;
      std::vector<float> points;

      request_to_mqtt["topicName"] = "/remoteops/on_road_pc";
      request_to_mqtt["msgType"] = "on_road_pc";

      uint64_t nanosec_ts =
          static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nsec;

      request_to_mqtt["stamp"] = nanosec_ts;

      for (size_t i = 0; i < point_cloud.points.size(); i++) {
        points.push_back(point_cloud.points[i].x);
        points.push_back(point_cloud.points[i].y);
        points.push_back(point_cloud.points[i].z);
      }

      request_to_mqtt["includesZ"] = true;
      request_to_mqtt["includesIntensity"] = false;
      request_to_mqtt["payload"] = points;

      publishToMqtt(request_to_mqtt);
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "1-onRoadPCCallback-Exception: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "2-onRoadPCCallback: Exception: " << e.what() << std::endl;
    }
  }
}

void RemoteHandler::curbPCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (getLinkupStatus()) {
    try
    {
      sensor_msgs::PointCloud point_cloud;
      // Convert the PointCloud2 to PointCloud
      if (!sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud)) {
        ROS_ERROR("Failed to convert PointCloud2 to PointCloud.");
        return;
      }

      nlohmann::json request_to_mqtt;
      std::vector<float> points;

      request_to_mqtt["topicName"] = "/remoteops/curb_pc";
      request_to_mqtt["msgType"] = "curb_pc";

      uint64_t nanosec_ts =
          static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nsec;

      request_to_mqtt["stamp"] = nanosec_ts;

      for (size_t i = 0; i < point_cloud.points.size(); i++) {
        points.push_back(point_cloud.points[i].x);
        points.push_back(point_cloud.points[i].y);
        points.push_back(point_cloud.points[i].z);
      }

      request_to_mqtt["includesZ"] = true;
      request_to_mqtt["includesIntensity"] = false;
      request_to_mqtt["payload"] = points;

      publishToMqtt(request_to_mqtt);
    }  
    catch (nlohmann::json::type_error& e) {
        std::cerr << "1-curbPCCallback-Exception: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr<<"2-curbPCCallback: Exception: " << e.what() << std::endl;
    }
  }
}
void RemoteHandler::mostConstrainedPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (getLinkupStatus()) 
  {
    try
    {
      sensor_msgs::PointCloud point_cloud;
    // Convert the PointCloud2 to PointCloud
      if (!sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud)) {
        ROS_ERROR("Failed to convert PointCloud2 to PointCloud.");
        return;
      }

      nlohmann::json request_to_mqtt;
      std::vector<float> points;

      request_to_mqtt["topicName"] = "/remoteops/most_constrained_points";
      request_to_mqtt["msgType"] = "most_constrained_points";

      uint64_t nanosec_ts =
          static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nsec;

      request_to_mqtt["stamp"] = nanosec_ts;

      for (size_t i = 0; i < point_cloud.points.size(); i++) {
        points.push_back(point_cloud.points[i].x);
        points.push_back(point_cloud.points[i].y);
        points.push_back(point_cloud.points[i].z);
      }

      request_to_mqtt["includesZ"] = true;
      request_to_mqtt["includesIntensity"] = false;
      request_to_mqtt["payload"] = points;

      publishToMqtt(request_to_mqtt);
  }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-mostConstrainedPointsCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "\n2-mostConstrainedPointsCallback-Exception[std]: " << e.what() << std::endl;
    }
    catch (const std::logic_error& e) {
        std::cerr << "\n3-mostConstrainedPointsCallback-Exception[logic_error]:" << e.what() << std::endl;
    }   
  }
}

void RemoteHandler::roadIntensityDetectionCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (getLinkupStatus()) 
  {
    try
    {
      sensor_msgs::PointCloud point_cloud;
      // Convert the PointCloud2 to PointCloud
      if (!sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud)) {
        ROS_ERROR("Failed to convert PointCloud2 to PointCloud.");
        return;
      }

      nlohmann::json request_to_mqtt;
      std::vector<float> points;

      request_to_mqtt["topicName"] = "/remoteops/road_intensity_detection";
      request_to_mqtt["msgType"] = "road_intensity_detection";

      uint64_t nanosec_ts =
          static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nsec;

      request_to_mqtt["stamp"] = nanosec_ts;

      for (size_t i = 0; i < point_cloud.points.size(); i++) {
        points.push_back(point_cloud.points[i].x);
        points.push_back(point_cloud.points[i].y);
        points.push_back(point_cloud.points[i].z);
      }

      request_to_mqtt["includesZ"] = true;
      request_to_mqtt["includesIntensity"] = false;
      request_to_mqtt["payload"] = points;

      publishToMqtt(request_to_mqtt);
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "1-roadIntensityDetectionCallback-Exception: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "2-roadIntensityDetectionCallback: Exception: " << e.what() << std::endl;
    }

  }
}

void RemoteHandler::ehmiInfoCallback(const std_msgs::String::ConstPtr& msg) {
  if (getLinkupStatus()) {
    try
    {
      std::string msg_str = msg->data;
      nlohmann::json request_to_mqtt;
     
      if (msg_str == "")
      {
        msg_str = " ";
      }

      if (prev_ehmi_info_ == msg_str)
      {
         return;
      }
     
      prev_ehmi_info_ = msg_str;
      request_to_mqtt["topicName"] = "/remoteops/ehmi_info";
      request_to_mqtt["msgType"] = "ehmi_info";
      request_to_mqtt["ehmiInfo"] = msg_str;
      publishToMqtt(request_to_mqtt);
    }
    catch (nlohmann::json::type_error& e) {
      std::cerr << "\n1-ehmiInfoCallback-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
      std::cerr << "\n2-ehmiInfoCallback-Exception[std]:" << e.what() << std::endl;
    }
    catch (const std::logic_error& e) {
        std::cerr << "\n3-ehmiInfoCallback-Exception[logic_error]:" << e.what() << std::endl;
    }  
  }
}

uint8_t RemoteHandler::getAiosContainerType(uint8_t psaCntType)
{
  uint8_t aios_cnt_type;
 
  if (psaCntType == static_cast<uint8_t>(PsaContainerTypeEnum::CONTAINER_TYPE_40FT)) 
  {
    aios_cnt_type = static_cast<uint8_t>(AiosContainerTypeEnum::CNT_40FT);
  }
  else if (psaCntType == static_cast<uint8_t>(PsaContainerTypeEnum::CONTAINER_TYPE_45FT)) 
  {
    aios_cnt_type = static_cast<uint8_t>(AiosContainerTypeEnum::CNT_45FT);
  }
  else if (psaCntType == static_cast<uint8_t>(PsaContainerTypeEnum::CONTAINER_TYPE_20FT_FRONT)) 
  {
    aios_cnt_type = static_cast<uint8_t>(AiosContainerTypeEnum::CNT_20FT_FRONT);
  }
  else if (psaCntType == static_cast<uint8_t>(PsaContainerTypeEnum::CONTAINER_TYPE_20FT_REAR)) 
  {
    aios_cnt_type = static_cast<uint8_t>(AiosContainerTypeEnum::CNT_20FT_REAR);
  }
  else if (psaCntType == static_cast<uint8_t>(PsaContainerTypeEnum::CONTAINER_TYPE_20FT_TWIN)) 
  {
    aios_cnt_type = static_cast<uint8_t>(AiosContainerTypeEnum::CNT_20FT_TWIN);
  }

  return aios_cnt_type;
};

uint8_t RemoteHandler::getAiosJobType(uint8_t psaJobType)
{
  uint8_t aios_job_type;

  if (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_OFFLOADING))  
  {
     aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::OFF_LOADING);
  }
  else if (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_MOUNTING))  
  {
     aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::LOADING);
  }
  else if (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_NO_YARD_ALLOCATION))  
  {
    aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::END_SLOT_STANDBY);
  }
  else if (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_STANDBY))
  {
     aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::STANDBY);
  }
  else if (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_MAINTENANCE))  
  {
     aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::MAINTENANCE);
  }
  else if (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_REFUELING)) 
  {
     aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::FUEL);
  }
   else if (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_MANUAL_ROUTE)) 
  {
     aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::MANUAL_ROUTE);
  }
  else if  (psaJobType == static_cast<uint8_t>(PsaJobTypeEnum::JOB_TYPE_PARKING)) 
  {
     aios_job_type =  static_cast<uint8_t>(AiosJobTypeEnum::PARKING);
  }
   
  return aios_job_type;

};


void RemoteHandler::initializeMqttComms() {
  // ROS_WARN_STREAM("Initializing MQTT");
  try
  {
    json_aios_mqtt_config_["type"] = "configuration";
    json_aios_mqtt_config_["client_id"] = client_id_;
    json_aios_mqtt_config_["address"] = broker_address_;
    json_aios_mqtt_config_["timeout"] = timeout_sec_;
    json_aios_mqtt_config_["sub_ns"] = mqtt_sub_topic_namespace_;
    json_aios_mqtt_config_["pub_ns"] = mqtt_pub_topic_namespace_;
    json_aios_mqtt_config_["sub_topic"] = nlohmann::json::array();
    json_aios_mqtt_config_["pub_topic"] = nlohmann::json::array();
    json_aios_mqtt_config_["debug"] = debug_;

    for (const auto& j : json_msg_files_) {
      std::string mqttSubTopicName = j["fromMqttTopicName"];
      std::string mqttPubTopicName = j["toMqttTopicName"];

      nlohmann::json json_mqtt_pubtopic;
      nlohmann::json json_mqtt_subtopic;

      if (mqttSubTopicName != "") {
        json_mqtt_subtopic["mqtt_sub_topic_name"] = mqttSubTopicName;
        json_aios_mqtt_config_["sub_topic"].push_back(json_mqtt_subtopic);
      }

      if (mqttPubTopicName != "") {
        json_mqtt_pubtopic["mqtt_pub_topic_name"] = mqttPubTopicName;
        json_aios_mqtt_config_["pub_topic"].push_back(json_mqtt_pubtopic);
      }
    }
    std_msgs::String config_payload;
    config_payload.data = json_aios_mqtt_config_.dump();
    // std::cout << "Config data:" << config_payload.data;
    pub_to_mqtt_.publish(config_payload);

    mqtt_initialized_ = true;
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-initializeMqttComms-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
        std::cerr<<"\n2-initializeMqttComms-Exception[std]:" << e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
        std::cerr << "\n3-initializeMqttComms-Exception[logic_error]:" << e.what() << std::endl;
  }    
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "\n4-initializeMqttComms-Exception[parse error]:"<< e.what() << std::endl;
  }  

}

/************************************************************************************************************
 * Function to parse json files named from launch file to respective json variables
 ************************************************************************************************************/
void RemoteHandler::parseJsonFile(std::string param_name) {
  try
  {
    std::string file_json_from_param;
    nlohmann::json j;
    private_nh_.param<std::string>(param_name, file_json_from_param, "");
    if (file_json_from_param != "") {
      std::ifstream ifs_json_file(file_json_from_param);
      j = nlohmann::json::parse(ifs_json_file);
    }
    if (!j.empty()) {
      int index = json_msg_files_.size();
      json_msg_files_.push_back(j);
      if (j["fromMqttTopicName"] != "") {
        std::string mqttSubTopic = j["fromMqttTopicName"];
        json_sub_topic_mapping_[mqttSubTopic] = index;
      }
      if (j["toMqttTopicName"] != "") {
        std::string mqttPubTopic = j["toMqttTopicName"];
        json_pub_topic_mapping_[mqttPubTopic] = index;
      }
    }
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-parseJsonFile-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
        std::cerr<<"\n2-parseJsonFile-Exception[std]:" << e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
        std::cerr << "\n3-parseJsonFile-Exception[logic_error]:" << e.what() << std::endl;
  }    
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "\n4-parseJsonFile-Exception[parse error]:"<< e.what() << std::endl;
  }  
}


/************************************************************************************************************
 * Publish payload to MQTT client
 ************************************************************************************************************/
void RemoteHandler::publishToMqtt(nlohmann::json& root_msg) {
  try
  {
    std_msgs::String payload;
    payload.data = root_msg.dump();
    pub_to_mqtt_.publish(payload);  /// aios/remote_handler/mqtt/to_client
  } 
  catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-publishToMqtt-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
        std::cerr<<"\n2-publishToMqtt-Exception[std]:" << e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
        std::cerr << "\n3-publishToMqtt-Exception[logic_error]:" << e.what() << std::endl;
  }    
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "\n4-publishToMqtt-Exception[parse error]:"<< e.what() << std::endl;
  } 
}

void RemoteHandler::sendRemoteRequest(std::string msg_type) {
  try
  {
    int index = json_pub_topic_mapping_[msg_type];

    nlohmann::json json_msg_file = json_msg_files_[index];
    nlohmann::json request_to_mqtt;
    std::string to_mqtt_topic_name = json_msg_file["toMqttTopicName"];

    if (to_mqtt_topic_name != "") {
      request_to_mqtt["topicName"] = mqtt_pub_topic_namespace_ + to_mqtt_topic_name;
      request_to_mqtt["msgType"] = msg_type;

      if (msg_type == "ping" && getLinkupStatus()) {
        request_to_mqtt["sender"] = linkup_vehicleID_;
        request_to_mqtt["receiver"] = linkup_consoleID_;
        request_to_mqtt["seq"] = send_ping_seq_;

        publishToMqtt(request_to_mqtt);

        last_ping_msg_timestamp_ = getTimestampNanosec();
        last_sent_ping_seq_ = send_ping_seq_;
        ping_ack_status_ = false;
        is_first_ping_.store(false);
      }

      else if (msg_type == "breaklink" && getLinkupStatus()) {
        request_to_mqtt["consoleID"] = linkup_consoleID_;
        request_to_mqtt["vehicleID"] = linkup_vehicleID_;
        request_to_mqtt["breakLinkReason"] = 1;  // timeout
        publishToMqtt(request_to_mqtt);
        ROS_ERROR("********VEHICLE SENDING BREAKLINK (TIMEOUT)******");

        linkup_status_.store(false);
        ROS_ERROR_STREAM("LinkUpStatus: FALSE");
        publishLinkupStatus("FALSE - VEHICLE BREAKLINK REQUEST(TIMEOUT)");
      }
    }
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-sendRemoteRequest-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
        std::cerr<<"\n2-sendRemoteRequest-Exception[std]:" << e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
        std::cerr << "\n3-sendRemoteRequest-Exception[logic_error]:" << e.what() << std::endl;
  }    
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "\n4-sendRemoteRequest-Exception[parse error]:"<< e.what() << std::endl;
  } 

}

void RemoteHandler::setErrorReset()
{
  try
  {
    std_msgs::String pub_msg;
    nlohmann::json payload;
    payload["error_reset"] = error_reset_request_;

    error_reset_request_ = !error_reset_request_;

    nlohmann::json msg;
    msg["timestamp"] = getTimestampMillisec();
    msg["type"] = "operation_cmd_override";
    msg["source"] = instruction_source_;
    msg["payload"] = payload;

    nlohmann::json ins_msg;
    ins_msg["timestamp"] = getTimestampMillisec();
    ins_msg["instructions"].push_back(msg);
    
    pub_msg.data = ins_msg.dump();
    pub_to_aios_instructions_.publish(pub_msg);



  }  
  catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-setErrorReset-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
        std::cerr<<"\n2-setErrorReset-Exception[std]:" << e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
        std::cerr << "\n3-setErrorReset-Exception[logic_error]:" << e.what() << std::endl;
  }    
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "\n4-setErrorReset-Exception[parse error]:"<< e.what() << std::endl;
  } 

}

void RemoteHandler::publishLinkupStatus(std::string status)
{
  std_msgs::String pub_linkup_msg;
  pub_linkup_msg.data = status;
  pub_to_linkup_status_.publish(pub_linkup_msg);
}

void RemoteHandler::publishRemoteOpsInit()
{
  //Parse JSON file
  try
  {
    std::string file_json_from_param;
    nlohmann::json j;
    private_nh_.param<std::string>("alert_metric_json_file", file_json_from_param, "");
    if (file_json_from_param != "") {
      std::ifstream ifs_json_file(file_json_from_param);
      j = nlohmann::json::parse(ifs_json_file);
    }
    if (!j.empty()) 
    {
       if (j.contains("alerts"))
       {
        std::vector<nlohmann::json> alerts_vec = j["alerts"].get<std::vector<nlohmann::json>>();
        std::vector<nlohmann::json> metrics_vec = j["metrics"].get<std::vector<nlohmann::json>>();
        std::vector<nlohmann::json> alert_metric_vec = j["alertMetric"].get<std::vector<nlohmann::json>>();

        for (size_t i = 0; i < alerts_vec.size(); i++)
        {
          int id = alerts_vec[i]["id"];
          std::string desc = alerts_vec[i]["description"];
          std::string type = alerts_vec[i]["type"];
          nlohmann::json obj;
          obj["description"] = desc;
          obj["type"] = type;
          alerts_map_[id] = obj;
        }

        for (size_t i = 0; i < metrics_vec.size(); i++)
        {
          int id = metrics_vec[i]["id"];
          std::string desc = metrics_vec[i]["description"];
          int valueType = metrics_vec[i]["valueType"];
          nlohmann::json obj;
          obj["description"] = desc;
          obj["valueType"] = valueType;
          metrics_map_[id] = obj;
        }

        for (size_t i = 0; i < alert_metric_vec.size(); i++)
        { 
          int alert_id = alert_metric_vec[i]["alertID"];
          int metric_id = alert_metric_vec[i]["metricID"];
          std::string threshold = alert_metric_vec[i]["threshold"];
          nlohmann::json obj;
          obj["metricID"] = metric_id;
          obj["threshold"] = threshold;
          alert_metric_map_[alert_id] = obj;
        }
      
     
    }
    }
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-publishRemoteOpsInit-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
        std::cerr<<"\n2-publishRemoteOpsInit-Exception[std]:" << e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
        std::cerr << "\n3-publishRemoteOpsInit-Exception[logic_error]:" << e.what() << std::endl;
  }    
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "\n4-publishRemoteOpsInit-Exception[parse error]:"<< e.what() << std::endl;
  }  
  //send remote_ops_init
  nlohmann::json ops_init_msg;
  std::string remote_ops_init_topic = "/remoteops/remote_ops_init"; 
  std::string  remote_ops_init_msg_type = "remote_ops_init";
  ops_init_msg["topicName"] = remote_ops_init_topic;
  ops_init_msg["msgType"] = remote_ops_init_msg_type;
  
  for (const auto& el:alerts_map_)
  {
     uint32_t alert_id = static_cast<uint32_t>(el.first);
     ops_init_msg["alertIds"].push_back(alert_id);
    
     std::string alert_desc = el.second["description"];
     ops_init_msg["alertDescriptions"].push_back(alert_desc);
  }
  for (const auto& el:alert_metric_map_)
  {
     int alert_id = static_cast<uint32_t>(el.first);
     ops_init_msg["alertType"].push_back(static_cast<uint32_t>(alert_id));
    
     int metric_type = el.second["metricID"];
     ops_init_msg["metricsType"].push_back(static_cast<uint8_t>(metric_type));
     
     int metric_value_type =  metrics_map_[metric_type]["valueType"];
     ops_init_msg["metricsValueType"].push_back(static_cast<uint8_t>(metric_value_type));
    
     std::string alert_threshold = el.second["threshold"];
     ops_init_msg["alertThreshold"].push_back(alert_threshold);
    

  }
  ROS_WARN_STREAM("RemoteOpsInit:"<<ops_init_msg.dump());
      
  publishToMqtt(ops_init_msg);

  std_msgs::String pub_msg;
  pub_msg.data = ops_init_msg.dump();
  pub_to_remote_ops_init_.publish(pub_msg);
}

void RemoteHandler::autoResume()
{
  std_msgs::String pub_msg;
  nlohmann::json payload;
  payload["force_pause_resume"] = static_cast<uint8_t>(AiosPauseResumeStatusEnum::RESUME);
  //pause_resume_ins_flag_.store(true); 
  isPause_.store(false);
  ROS_WARN_STREAM("[remote_handler]autoResume");
  nlohmann::json msg;
  msg["timestamp"] = getTimestampNanosec();
  msg["type"] = "control_cmd_override";
  msg["source"] = instruction_source_;
  msg["payload"] = payload;

  nlohmann::json ins_msg;
  ins_msg["timestamp"] = getTimestampMillisec();
  ins_msg["instructions"].push_back(msg);

  pub_msg.data = ins_msg.dump();
  pub_to_aios_instructions_.publish(pub_msg);
}

void RemoteHandler::handleRemoteConsoleRequest(const std::string& msg_type,
                                               const nlohmann::json& remote_response_msg) {
  
  try
  {
    int index = json_sub_topic_mapping_[msg_type];
    nlohmann::json json_msg_file = json_msg_files_[index];
    nlohmann::json response_to_mqtt;
    std::string to_mqtt_topic_name = json_msg_file["toMqttTopicName"];

    if (msg_type == "linkup") {
      if (to_mqtt_topic_name != "") {
        response_to_mqtt["topicName"] = mqtt_pub_topic_namespace_ + to_mqtt_topic_name;
        response_to_mqtt["msgType"] = msg_type;
        response_to_mqtt["vehicleID"] = remote_response_msg["payload"]["vehicleID"];
        response_to_mqtt["consoleID"] = remote_response_msg["payload"]["consoleID"];
        response_to_mqtt["viewType"] = remote_response_msg["payload"]["viewType"];
        response_to_mqtt["result"] = json_msg_file["LinkUpResult"]["success"];
        response_to_mqtt["resultText"] = "success";
        linkup_vehicleID_ = response_to_mqtt["vehicleID"];
        linkup_consoleID_ = response_to_mqtt["consoleID"];

        publishToMqtt(response_to_mqtt);
        publishRemoteOpsInit();
 
                 
     //initializations
        nudge_joystick_release_.clear();  
        nudge_joystick_push_.clear();
        prevNudgeInstanceId_ = -1;

        prev_avcs_destination_from_ui_ = "";
        prev_ehmi_info_ = "";
        prev_avcs_job_status_ = -1;

        dest_first_link_up_ = true;
        job_status_first_link_up_ = true;

        is_first_ping_.store(true);
        ping_response_latency_ = 0;
        ping_request_latency_ = 0; 
        last_console_ack_sent_timestamp_= 0;
        send_ping_seq_ = 0;
        last_sent_ping_seq_ = 0;
        
        linkup_status_.store(true);
        ROS_WARN("LINKUP STATUS: TRUE");
        publishLinkupStatus("TRUE");
      }

    } else if (msg_type == "teardown") {
      if (to_mqtt_topic_name != "") {
        response_to_mqtt["topicName"] = mqtt_pub_topic_namespace_ + to_mqtt_topic_name;
        response_to_mqtt["msgType"] = msg_type;
        response_to_mqtt["vehicleID"] = remote_response_msg["payload"]["vehicleID"];
        response_to_mqtt["consoleID"] = remote_response_msg["payload"]["consoleID"];
        response_to_mqtt["result"] = json_msg_file["TeardownResult"]["success"];
        response_to_mqtt["resultText"] = "success";
        publishToMqtt(response_to_mqtt);
        linkup_status_.store(false);
        ROS_ERROR("LINKUP STATUS: FALSE");
        publishLinkupStatus("FALSE - CONSOLE TEARDOWN REQUEST");
       
      }

    } else if (msg_type == "breaklink") {
      if (to_mqtt_topic_name != "") {
        if (remote_response_msg["payload"].contains("breakLinkReason")) {
          response_to_mqtt["topicName"] = mqtt_pub_topic_namespace_ + to_mqtt_topic_name;
          response_to_mqtt["msgType"] = msg_type;
          response_to_mqtt["vehicleID"] = remote_response_msg["payload"]["vehicleID"];
          response_to_mqtt["consoleID"] = remote_response_msg["payload"]["consoleID"];
          publishToMqtt(response_to_mqtt);
          linkup_status_.store(false);
          ROS_ERROR("LINKUP STATUS: FALSE");
          publishLinkupStatus("FALSE - CONSOLE BREAKLINK REQUEST");
        }
      }

    } else if (msg_type == "ping") {
      // vehicle sending ack msg to console.
      if ((to_mqtt_topic_name != "") && (remote_response_msg.contains("calculatedRequestLatency"))) {
        return;
      } else  // vehicle got console ping ack
      {
        int seq = remote_response_msg["payload"]["seq"];

        if (seq == last_sent_ping_seq_) {
          ping_request_latency_ = remote_response_msg["payload"]["requestLatency"];
          ping_response_latency_ = remote_response_msg["response_latency_ms"];
          last_console_ack_sent_timestamp_ = remote_response_msg["console_ack_sent_ts"];

          ping_ack_status_ = true;
          // increasing seq after getting ack from console
          send_ping_seq_++;
        }
      }
    } else if (msg_type == "emergency_brake_command") {
     
      std_msgs::String pub_msg;
      
      nlohmann::json payload;
      bool state = remote_response_msg["payload"]["applyBrake"];
      payload["emergency_brake"] = state;
      emergency_brake_on_.store(state ? (EmergencyBrakeState::ON) : (EmergencyBrakeState::OFF));
      
      nlohmann::json msg;
      msg["timestamp"] = remote_response_msg["timestamp"];
      msg["type"] = "control_cmd_override";
      msg["source"] = instruction_source_;
      msg["payload"] = payload;

      nlohmann::json ins_msg;
      ins_msg["timestamp"] = getTimestampMillisec();
      ins_msg["instructions"].push_back(msg);
    
      pub_msg.data = ins_msg.dump();
      pub_to_aios_instructions_.publish(pub_msg);

    } else if (msg_type == "cmd_signal_light") {
  
      std_msgs::String pub_msg;

      nlohmann::json payload;

      int state = remote_response_msg["payload"]["state"];

      ROS_WARN_STREAM("Console sent Indicator_Light: "<<state);

      if (state == static_cast<uint8_t>(PsaSignalLightStatusEnum::BOTH_SIGNAL_LIGHTS_OFF)) {
        payload["indicator_light"] =
            static_cast<uint8_t>(AiosSignalLightStatusEnum::BOTH_SIGNAL_LIGHTS_OFF);
        payload["hazard_light"] =
            static_cast<uint8_t>(AiosHazardLightStatusEnum::OFF);      
      
      } else if (state == static_cast<uint8_t>(PsaSignalLightStatusEnum::RIGHT_SIGNAL_LIGHT_ON)) {
        payload["indicator_light"] =
            static_cast<uint8_t>(AiosSignalLightStatusEnum::RIGHT_SIGNAL_LIGHT_ON);
      
      } else if (state == static_cast<uint8_t>(PsaSignalLightStatusEnum::LEFT_SIGNAL_LIGHT_ON)) {
        payload["indicator_light"] =
            static_cast<uint8_t>(AiosSignalLightStatusEnum::LEFT_SIGNAL_LIGHT_ON);
      
      }
      else if (state == static_cast<uint8_t>(
                            PsaSignalLightStatusEnum::BOTH_SIGNAL_LIGHTS_ON))  // hazard lights
      {
        payload["indicator_light"] =
            static_cast<uint8_t>(AiosSignalLightStatusEnum::BOTH_SIGNAL_LIGHTS_ON);
        
        payload["hazard_light"] =
            static_cast<uint8_t>(AiosHazardLightStatusEnum::ON);       
      }

      nlohmann::json msg;
      msg["timestamp"] = remote_response_msg["timestamp"];
      msg["type"] = "aux_cmd_override";
      msg["source"] = instruction_source_;
      msg["payload"] = payload;

      nlohmann::json ins_msg;
      ins_msg["timestamp"] = getTimestampMillisec();
      ins_msg["instructions"].push_back(msg);

      pub_msg.data = ins_msg.dump();

      pub_to_aios_instructions_.publish(pub_msg);

    } else if (msg_type == "cmd_head_light_remote") {
    
      std_msgs::String pub_msg;
      
      int state = remote_response_msg["payload"]["state"];

      nlohmann::json payload;

      if (state == static_cast<uint8_t>(PsaHeadLightStatusEnum::LOW_BEAM)) {
        payload["head_light"] = static_cast<uint8_t>(AiosHeadLightStatusEnum::LOW_BEAM);
      } else if (state == static_cast<uint8_t>(PsaHeadLightStatusEnum::HIGH_BEAM)) {
        payload["head_light"] = static_cast<uint8_t>(AiosHeadLightStatusEnum::HIGH_BEAM);
      } else if (state == static_cast<uint8_t>(PsaHeadLightStatusEnum::OFF)) {
        payload["head_light"] = static_cast<uint8_t>(AiosHeadLightStatusEnum::OFF);
      }

    nlohmann::json msg;
    msg["timestamp"] = remote_response_msg["timestamp"];
    msg["type"] = "aux_cmd_override";
    msg["source"] = instruction_source_;
    msg["payload"] = payload;
    
    nlohmann::json ins_msg;
    ins_msg["timestamp"] = getTimestampMillisec();
    ins_msg["instructions"].push_back(msg);

    pub_msg.data = ins_msg.dump();
    pub_to_aios_instructions_.publish(pub_msg);

    } else if (msg_type == "cmd_horn") {

      std_msgs::String pub_msg;
      
      nlohmann::json payload;
      bool activateHorn = remote_response_msg["payload"]["activateHorn"];

      if (activateHorn) {
        payload["horn"] = static_cast<uint8_t>(AiosHornEnum::ON);
      } else {
        payload["horn"] = static_cast<uint8_t>(AiosHornEnum::OFF);
      }

      nlohmann::json msg;
      msg["timestamp"] = remote_response_msg["timestamp"];
      msg["type"] = "aux_cmd_override";
      msg["source"] = instruction_source_;
      msg["payload"] = payload;

      nlohmann::json ins_msg;
      ins_msg["timestamp"] = getTimestampMillisec();
      ins_msg["instructions"].push_back(msg);

      pub_msg.data = ins_msg.dump();
      pub_to_aios_instructions_.publish(pub_msg);
    
    } else if (msg_type == "traffic_light_override") {
      std_msgs::String pub_msg;
    
      int type = remote_response_msg["payload"]["type"];

      // Create payload
      // type @0 :UInt8; # [0 - Red light and Red turn right], [1 - Green light], [3 - Green turn right]
      nlohmann::json payload;
      if (type == 0) {
        payload["tl_signal_state"] = static_cast<uint8_t>(AiosTrafficLightStatusEnum::RED);
      } else if (type == 1) {
        payload["tl_signal_state"] = static_cast<uint8_t>(AiosTrafficLightStatusEnum::GREEN);
      } else if (type == 3) {
        payload["tl_signal_state"] =
            static_cast<uint8_t>(AiosTrafficLightStatusEnum::GREEN_RIGHT_ARROW);
      }

     
      nlohmann::json msg;
      msg["timestamp"] = remote_response_msg["timestamp"];
      msg["type"] = "traffic_light_override";
      msg["source"] = instruction_source_;
      msg["payload"] = payload;
      
      nlohmann::json ins_msg;
      ins_msg["timestamp"] = getTimestampMillisec();
      ins_msg["instructions"].push_back(msg);
      
      pub_msg.data = ins_msg.dump();
      pub_to_aios_instructions_.publish(pub_msg);

    } else if (msg_type == "stop") {
      
      std_msgs::String pub_msg;
      
      bool pause = remote_response_msg["payload"]["pause"];

      nlohmann::json payload;
      if (pause) {
        payload["force_pause_resume"] = static_cast<uint8_t>(AiosPauseResumeStatusEnum::PAUSE);
        //pause_resume_ins_flag_.store(true); 
        isPause_.store(true);
        ROS_WARN_STREAM("Pause command from Remote Console");
      } else {
        payload["force_pause_resume"] = static_cast<uint8_t>(AiosPauseResumeStatusEnum::RESUME);
        //pause_resume_ins_flag_.store(true); 
        isPause_.store(false);
        ROS_WARN_STREAM("Resume command from Remote Console");
      }

      nlohmann::json msg;
      msg["timestamp"] = remote_response_msg["timestamp"];
      msg["type"] = "control_cmd_override";
      msg["source"] = instruction_source_;
      msg["payload"] = payload;

      nlohmann::json ins_msg;
      ins_msg["timestamp"] = getTimestampMillisec();
      ins_msg["instructions"].push_back(msg);

      pub_msg.data = ins_msg.dump();
      pub_to_aios_instructions_.publish(pub_msg);

    } else if (msg_type == "precedence_override") {

      std_msgs::String pub_msg;
    
      nlohmann::json payload;

      // Bool 0-stop 1-go
      bool isClearHazards = remote_response_msg["payload"]["go"];
      payload["clear_transit_hazards"] = isClearHazards;
      payload["clear_ego_hazards"] = isClearHazards;
      payload["clear_other_hazards"] = isClearHazards;

      nlohmann::json msg;
      msg["timestamp"] = remote_response_msg["timestamp"];
      msg["type"]      = "hazards_override";
      msg["source"]  = instruction_source_;
      msg["payload"] = payload;

      nlohmann::json ins_msg;
      ins_msg["timestamp"] = getTimestampMillisec();
      ins_msg["instructions"].push_back(msg);

      pub_msg.data = ins_msg.dump();
      pub_to_aios_instructions_.publish(pub_msg);

    } else if (msg_type == "goal") {
      try
      {
        autoResume();
        std_msgs::String pub_msg;
        
        nlohmann::json payload;

        std::string block_name = remote_response_msg["payload"]["blockName"];
        uint8_t block_id = remote_response_msg["payload"]["blockId"];
        uint8_t lane_id = remote_response_msg["payload"]["laneId"];
        uint8_t slot_id = remote_response_msg["payload"]["slotId"];
        uint8_t container_type = remote_response_msg["payload"]["containerType"];
        uint8_t aios_container_type = getAiosContainerType(container_type);
        uint8_t job_type = remote_response_msg["payload"]["jobType"];
        uint8_t aios_job_type = getAiosJobType(job_type);
            
        payload["block_id"] = block_id; 
        payload["block_name"] = block_name; 
        payload["lane_id"]  = lane_id; 
        payload["slot_id"]  = slot_id; 
        payload["container_type"] = aios_container_type; 
        payload["job_type"] = aios_job_type;             
      
        nlohmann::json msg;
        msg["timestamp"] = remote_response_msg["timestamp"];
        msg["type"] = "new_destination_request";
        msg["source"] = instruction_source_;
        msg["payload"] = payload;

        nlohmann::json ins_msg;
        ins_msg["timestamp"] = getTimestampMillisec();
        ins_msg["instructions"].push_back(msg);

        pub_msg.data = ins_msg.dump();
        pub_to_aios_instructions_.publish(pub_msg);
      }
      catch (const std::exception& e) {
        std::cerr<<"1-ERROR -  goal[handleRemoteConsoleRequest]:" << e.what()<<std::endl;
        return;
      }  
      catch (nlohmann::json::type_error& e) {
        std::cerr<<"2-ERROR -  goal[handleRemoteConsoleRequest]:" << e.what()<<std::endl;
        return;
      }

    } else if (msg_type == "adjust_position") {
      try
      {
        //we should auto resume..

        autoResume();

        std_msgs::String pub_msg;
      

        nlohmann::json payload;
        std::string msg_type;
        if (remote_response_msg["payload"].contains("distance")) {
          msg_type = "new_alignment_request";
          payload["alignment_mode"] = static_cast<uint8_t>(AiosAlignmentModeEnum::DISTANCE);

          double distance = remote_response_msg["payload"]["distance"];
          
          if (distance > 3.0)
          {
            distance = 3.0;
          }
          else if (distance < -3.0) 
          {
             distance = -3.0;
          }
          payload["remaining_distance"] = distance;

        }
        else if (remote_response_msg["payload"].contains("safetyOverride")) {
          msg_type = "hazards_override";

          bool safety_override = remote_response_msg["payload"]["safetyOverride"];

          payload["clear_transit_hazards"] = safety_override;
          payload["clear_ego_hazards"] = safety_override;
          payload["clear_other_hazards"] = safety_override;
        }

        nlohmann::json msg;
        msg["timestamp"] = remote_response_msg["timestamp"];
        msg["type"] = msg_type;
        msg["source"]= instruction_source_;
        msg["payload"] = payload;

        nlohmann::json ins_msg;
        ins_msg["timestamp"] = getTimestampMillisec();
        ins_msg["instructions"].push_back(msg);

        pub_msg.data = ins_msg.dump();
        pub_to_aios_instructions_.publish(pub_msg);


      }
    
      catch (const std::exception& e) {
        std::cerr<<"ERROR -  adjust_position[handleRemoteConsoleRequest-std::exception]:" << e.what()<<std::endl;
        return;
      }  
    
    } else if (msg_type == "non_yard_goal") {

      try
      {
        //container type 1, job type 5
        std_msgs::String pub_msg;

        uint32_t destination_id = remote_response_msg["payload"]["destination"];
        std::string aios_destination_id = getAiosDesinationtId(destination_id);
        if (aios_destination_id == "")
        {
          ROS_WARN_STREAM("Destination ignored:"<<destination_id);
          return;
        }
        
        
        nlohmann::json payload;
        
        payload["destination_id"] = aios_destination_id;
        payload["container_type"] = 1; //front
        payload["job_type"] = 5; //Standby

        nlohmann::json msg;
        msg["timestamp"] = remote_response_msg["timestamp"];
        msg["type"] = "new_destination_request";
        msg["source"] = instruction_source_;
        msg["payload"] = payload;

        nlohmann::json ins_msg;
        ins_msg["timestamp"] = getTimestampMillisec();
        ins_msg["instructions"].push_back(msg);
        
        pub_msg.data = ins_msg.dump();
        pub_to_aios_instructions_.publish(pub_msg);
      }  
      catch (const std::exception& e) {
        std::cerr<<"ERROR -  non_yard_goal[handleRemoteConsoleRequest-std::exception]:" << e.what()<<std::endl;
        return;
      }  
    } else if (msg_type == "location")  // relocalization
    {
      try
      {
        std_msgs::String pub_msg;
      
        double x   = remote_response_msg["payload"]["x"];
        double y   = remote_response_msg["payload"]["y"];
        double yaw = remote_response_msg["payload"]["yaw"]; //rad

        nlohmann::json payload;
        payload["correction_mode"] = 19;
        payload["given_frame"] = aid_map_frame_;
        nlohmann::json pose;
        pose["unit"] =  static_cast<uint8_t>(AiosPoseUnitEnum::UTM); //meters
        pose["pose2d"] = nlohmann::json::array();
        pose["pose2d"].push_back(x);
        pose["pose2d"].push_back(y);
        pose["pose2d"].push_back(yaw);
          
        payload["pose"] = pose;

        nlohmann::json msg;
        msg["timestamp"] = remote_response_msg["timestamp"];
        msg["type"] = "localization_override";
        msg["source"] = instruction_source_;
        msg["payload"] = payload;

        nlohmann::json ins_msg;
        ins_msg["timestamp"] = getTimestampMillisec();
        ins_msg["instructions"].push_back(msg);

        pub_msg.data = ins_msg.dump();
        pub_to_aios_instructions_.publish(pub_msg);
      }  
      catch (const std::exception& e) {
        std::cerr<<"ERROR -  location[handleRemoteConsoleRequest-std::exception]:" << e.what()<<std::endl;
        return;
      }  

    } else if (msg_type == "manual_push_remote")  // nudge_forward
    {
      try
      {
        //autoResume();
        std_msgs::String pub_msg;
      
        nlohmann::json payload;
        uint8_t engage = static_cast<uint8_t>(remote_response_msg["payload"]["engage"]);
        uint8_t nudge_instance_id = static_cast<uint8_t>(remote_response_msg["payload"]["nudgeInstanceId"]);
        //nudgeInstanceId of the command should be equal to or greater the latest nudgeInstanceId received
        if (static_cast<int>(nudge_instance_id) >= prevNudgeInstanceId_)
        {
        
          if (static_cast<int>(nudge_instance_id) == prevNudgeInstanceId_)
          {
            if (nudge_joystick_release_.count(nudge_instance_id) > 0) //already released 
            {
              ROS_WARN_STREAM("[NudgeInstanceID]:"<<static_cast<int>(nudge_instance_id)<<"Nudge cmd has been ignored.Operator already released joystick!");
              return;
            }

            if (nudge_joystick_push_.count(nudge_instance_id) > 0 && (engage != static_cast<uint8_t>(PsaNudgeModeEnum::RELEASE))) //already sent
            {
              return;
            }
            
          } 

          //TO DO-Safety check
          //check the latency of the message (i.e. the time the message was
          //created vs the time the vehicle received the message). 
          //The vehicle should then send an OpsResponse back to the vehicle 
          //based on whether the vehicle accepted or rejected the job.
        
          /***When operator lets go of the joystick, it sends 
              engage == 0 msg with same nudgeInstanceId ****/
          if (engage == static_cast<uint8_t>(PsaNudgeModeEnum::FORWARD))
          {
            payload["nudge"] = static_cast<uint8_t>(AiosNudgeModeEnum::FORWARD);
            nudge_joystick_push_[nudge_instance_id] = true;
          }
          else if (engage == static_cast<uint8_t>(PsaNudgeModeEnum::REVERSE))
          {
            payload["nudge"] = static_cast<uint8_t>(AiosNudgeModeEnum::REVERSE);
            nudge_joystick_push_[nudge_instance_id] = true;
          }
          else if (engage == static_cast<uint8_t>(PsaNudgeModeEnum::RELEASE))  // let go of the joystick
          {
            payload["nudge"] = static_cast<uint8_t>(AiosNudgeModeEnum::RELEASE);
            //after release, don't except any command with same nudgeInstanceId
            nudge_joystick_release_[nudge_instance_id] = true;
          }
          
          nlohmann::json msg;
          msg["timestamp"] = remote_response_msg["timestamp"];
          msg["type"] = "control_cmd_override";
          msg["source"] = instruction_source_;
          msg["payload"] = payload;
        
          nlohmann::json ins_msg;
          ins_msg["timestamp"] = getTimestampMillisec();
          ins_msg["instructions"].push_back(msg);

          pub_msg.data = ins_msg.dump();
          pub_to_aios_instructions_.publish(pub_msg);
              
          prevNudgeInstanceId_ = nudge_instance_id;
        }
      }
      catch (const std::exception& e) {
        std::cerr<<"ERROR -  manual_push_remote[handleRemoteConsoleRequest-std::exception]:" << e.what()<<std::endl;
        return;
      }    
    
    } 
    else if (msg_type == "override_path") 
    {
      try
      {
        std_msgs::String pub_msg;
      
        nlohmann::json payload;
        payload["segmentId"] = remote_response_msg["payload"]["segmentId"];
        payload["discretizedPathPointsX"]   = remote_response_msg["payload"]["discretizedPathPointsX"];
        payload["discretizedPathPointsY"]   = remote_response_msg["payload"]["discretizedPathPointsY"];
        payload["discretizedPathPointsYaw"] = remote_response_msg["payload"]["discretizedPathPointsYaw"];

        nlohmann::json msg;
        msg["timestamp"] = remote_response_msg["timestamp"];
        msg["type"] = "trajectory_override";
        msg["source"] = instruction_source_;
        msg["payload"] = payload;

        nlohmann::json ins_msg;
        ins_msg["timestamp"] = getTimestampMillisec();
        ins_msg["instructions"].push_back(msg);
        
        pub_msg.data = ins_msg.dump();
        pub_to_aios_instructions_.publish(pub_msg);
      }
      catch (const std::exception& e) {
        std::cerr<<"ERROR -  override_path[handleRemoteConsoleRequest-std::exception]:" << e.what()<<std::endl;
        return;
      }    
    }
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "\n1-handleRemoteConsoleRequest-Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
        std::cerr<<"\n2-handleRemoteConsoleRequest-Exception[std]:" << e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
        std::cerr << "\n3-handleRemoteConsoleRequest-Exception[logic_error]:" << e.what() << std::endl;
  }    
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "\n4-handleRemoteConsoleRequest-Exception[parse error]:"<< e.what() << std::endl;
  } 
}

void RemoteHandler::sendEgoStateMsgToConsole() {
    try 
    {
      ros::Rate loop_rate(ego_state_publish_rate_);
    
      nlohmann::json request_to_mqtt;
      request_to_mqtt["topicName"] = "/remoteops/ego_state";
      request_to_mqtt["msgType"] = "ego_state";


      //not used
      request_to_mqtt["parkingBrakeStatus"] = false;
      request_to_mqtt["observedSpeedLimit"] = 0.0;
      request_to_mqtt["inLaneChange"] = false;
      request_to_mqtt["gpsInMapX"] = 0.0;
      request_to_mqtt["gpsInMapY"] = 0.0;
      request_to_mqtt["precedenceOverrideInProgress"] = false;
      request_to_mqtt["passedPointOfNoReturn"] = false;
      request_to_mqtt["inSSA"] = false;
      int aios_apm_status;   
    
      while (ros::ok() && run_thread_ego_.load()) {
    
      if ( getLinkupStatus() )
      { 
          int v_size = 0;
          {
            std::lock_guard<std::mutex> lock(apm_preprocess_mtx_);
            v_size = vec_send_ego_state_.size();
          }     

          if ( v_size > 0 ) {
        
            aios_apm_msgs::AiosPreprocess apm_msg;
            
            {
              std::lock_guard<std::mutex> lock(apm_preprocess_mtx_);
              apm_msg = vec_send_ego_state_.front();
              vec_send_ego_state_.erase(vec_send_ego_state_.begin());
            }

            aios_apm_status = apm_msg.aidc_info.aidc_apm_status;
            request_to_mqtt["apmStatus"] = aios_apm_status; 

            auto aios_session_job_status = apm_msg.session_info.session_management_job_status;
            request_to_mqtt["sessionJobStatus"] = aios_session_job_status; 

            auto lane_type = apm_msg.ontology_info.lane_type;

            
            auto aios_hazard_light_status = apm_msg.airs_info.hazard_light_status;
            auto aios_gear_status = apm_msg.airs_info.gear_feedback;

            //SIGNAL LIGHT FEEDBACK(hazard)
            uint8_t signal_light;
          
            if (aios_hazard_light_status == 1) 
            {
              signal_light = 3; //Both on Hazard light
            }
            else 
            {
              signal_light = 0; //Both off
            }
            
            request_to_mqtt["signalLight"] = signal_light;

            //GEAR FEEDBACK
            uint8_t gear_status;
            if (aios_gear_status == 0)  // Neutral
            {
              gear_status = 3;
            } else if (aios_gear_status == 1)  // Drive
            {
              gear_status = 4;
            } else if (aios_gear_status == 2)  // Reverse
            {
              gear_status = 2;
            }
            request_to_mqtt["gearStatus"] = gear_status; 
            
            
             std::string target_frame = aid_map_frame_;
            //VEHICLE POSE FEEDBACK
            // request_to_mqtt["vehPositionX"] =  apm_msg.aide_info.current_pose_gl_sitemap.x; 
            // request_to_mqtt["vehPositionY"] =  apm_msg.aide_info.current_pose_gl_sitemap.y;
            // request_to_mqtt["vehYaw"]       =  apm_msg.aide_info.current_pose_gl_sitemap.theta;// rad
            
           
            try
            {
                 
              tf2_ros::TransformListener tf_listener(tf_buffer_);
              geometry_msgs::Pose2D ego_state_vehicle_pose;
              aios_lib::transform::populatePose(ego_state_vehicle_pose,"map_server_origin","gnss_link", &tf_buffer_);
              //ROS_WARN_STREAM("populatePose: vehX:"<< ego_state_vehicle_pose.x<<" vehY:"<<ego_state_vehicle_pose.y<<" yaw:"<<ego_state_vehicle_pose.theta);  
          
              request_to_mqtt["vehPositionX"] = ego_state_vehicle_pose.x; 
              request_to_mqtt["vehPositionY"] =  ego_state_vehicle_pose.y; 
              request_to_mqtt["vehYaw"]       =  ego_state_vehicle_pose.theta;
            }
            catch (std::runtime_error& e)
            {
                ROS_ERROR("sendEgoStateMsgToConsole-Error [ vehicle_transformed_pose ]: %s", e.what());
            }
            
            //TRAILER POSE FEEDBACK
            // request_to_mqtt["trailerPositionX"] = apm_msg.aipe_info.trailer_pose_sitemap.x;;
            // request_to_mqtt["trailerPositionY"] = apm_msg.aipe_info.trailer_pose_sitemap.y;
            // request_to_mqtt["trailerYaw"]       = apm_msg.aipe_info.trailer_pose_sitemap.theta; //rad

     
            geometry_msgs::Pose2D ego_state_trailer_pose;
            aios_lib::transform::populatePose(ego_state_trailer_pose,"map_server_origin","base_link", &tf_buffer_);
            //ROS_WARN_STREAM("trailer populatePose: vehX:"<<ego_state_trailer_pose.x<<" vehY:"<<ego_state_trailer_pose.y<<" yaw:"<<ego_state_trailer_pose.theta);  
            request_to_mqtt["trailerPositionX"] = ego_state_trailer_pose.x; 
            request_to_mqtt["trailerPositionY"] =  ego_state_trailer_pose.y; 
            request_to_mqtt["trailerYaw"]       = ego_state_trailer_pose.theta;
              
          
            request_to_mqtt["brakePercentage"] = apm_msg.airs_info.brake_feedback;  // (0 -100%)
          
          //VELOCITY FEEDBACK
            request_to_mqtt["velocity"] =
                static_cast<float>(apm_msg.airs_info.speed_feedback *
                                  static_cast<float>(5.0 / 18.0));  // (kmph) convert to m/s
            //STEERING FEEDBACK
            request_to_mqtt["steeringAngle"] =
                apm_msg.airs_info.steer_feedback;  //(dgrees +ve right, -ve left)
          
            //HEAD LIGHT FEEDBACK
            auto head_light_status =  apm_msg.airs_info.head_light_status;
          
            if ( head_light_status == 0 ) //OFF
            {
              request_to_mqtt["headLights"] = 0;
            }
            else if ( head_light_status == 1 ) //LOW
            {
              request_to_mqtt["headLights"] = 1;
            }
            else if ( head_light_status == 2 ) //HIGH
            {
              request_to_mqtt["headLights"] = 2;
            }
          
          
          //THROTTLE PERCENTAGE FEEDBACK
            request_to_mqtt["throttlePercentage"] = apm_msg.airs_info.throttle_feedback;
          
          //LOCALIZATION STATUS FEEDBACK
            if (static_cast<uint8_t>(localization_status_) == static_cast<uint8_t>(AiosLocalizationStatusEnum::GOOD))
            {
              request_to_mqtt["localizationStatus"] = static_cast<uint8_t>(PsaLocalizationStatusEnum::GOOD);
            }
            else if (static_cast<uint8_t>(localization_status_) == static_cast<uint8_t>(AiosLocalizationStatusEnum::POOR1))
            {
              request_to_mqtt["localizationStatus"] = static_cast<uint8_t>(PsaLocalizationStatusEnum::POOR);
            }
            else if (static_cast<uint8_t>(localization_status_) == static_cast<uint8_t>(AiosLocalizationStatusEnum::POOR2))
            {
              request_to_mqtt["localizationStatus"] = static_cast<uint8_t>(PsaLocalizationStatusEnum::POOR);
            }
            else if (static_cast<uint8_t>(localization_status_) == static_cast<uint8_t>(AiosLocalizationStatusEnum::CRITICAL))
            {
              request_to_mqtt["localizationStatus"] = static_cast<uint8_t>(PsaLocalizationStatusEnum::CRITICAL);
            }
            else 
            {
              ROS_WARN_STREAM_ONCE("ISSUE: Localization status val:"<<static_cast<uint8_t>(localization_status_));
             
              request_to_mqtt["localizationStatus"] = static_cast<uint8_t>(PsaLocalizationStatusEnum::CRITICAL);

            }
            //HORN STATUS FEEDBACK
            try
            {
              if (apm_msg.airs_info.horn_status)//can be null
              {
                request_to_mqtt["hornCmdFeedback"] = true; 
              }
              else 
              {
                request_to_mqtt["hornCmdFeedback"] = false;
              }
              
            }
            catch (const std::exception& e) 
            {
              std::cerr<<"Horn feedback error:"<<apm_msg.airs_info.horn_status<< "- "<< e.what()<<std::endl;
              request_to_mqtt["hornCmdFeedback"] = false;
            }
            
            //EMERGENCY BUTTON FEEDBACK
            if (emergency_brake_on_.load() == EmergencyBrakeState::INITIAL) //no cmd received yet
            {
              request_to_mqtt["remoteEmergencyButtonStatus"] = false; 
            }
            //GOT E-BRAKE ON CMD
            else if (emergency_brake_on_.load() == EmergencyBrakeState::ON) // Recieved TURN ON command
            {
              if (apm_msg.airs_info.safety_estop == 1)//SUCCESS
              {
                  request_to_mqtt["remoteEmergencyButtonStatus"] = true; 
                  emergency_brake_on_.store( EmergencyBrakeState::WAITING );
                  ROS_WARN("(1) emergency brake ON");
              } 
              
              else 
              {
                request_to_mqtt["remoteEmergencyButtonStatus"] = false; //FAIL
                ROS_WARN("(2-FAIL!!) emergency brake NOT ON");
              }
            }
            else if (emergency_brake_on_.load() == EmergencyBrakeState::OFF) //Recieved RELEASE command
            {
              if (apm_msg.airs_info.safety_estop == 0)//SUCCESS
              {
                  request_to_mqtt["remoteEmergencyButtonStatus"] = false; 
                  ROS_WARN("(3)Emergency brake OFF");
                  emergency_brake_on_.store( EmergencyBrakeState::WAITING );
                  error_reset_timer_ = ros::Timer();  // Setting to a default (empty) Timer object stops it
              } 
              else 
              {
                request_to_mqtt["remoteEmergencyButtonStatus"] = true; //FAIL
                ROS_WARN("ErrorReset to Release E-Brake");
              //  setErrorReset();
                
                if (!error_reset_timer_.hasStarted())
                {
                  error_reset_timer_ = nh_.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&) {
                        setErrorReset();  // Calls the method without any parameters
                    });
                }
               
              }

            }

          
          publishToMqtt(request_to_mqtt);
          //Publish to ROS Topic for debug
          try
          {
            //publish to ego_state topic
            aios_apm_msgs::EgoState ego_state_msg;
            ego_state_msg.signal_light = request_to_mqtt["signalLight"];
            ego_state_msg.veh_position_x = request_to_mqtt["vehPositionX"];
            ego_state_msg.veh_position_y = request_to_mqtt["vehPositionY"];
            ego_state_msg.veh_yaw = request_to_mqtt["vehYaw"];
            ego_state_msg.trailer_position_x = request_to_mqtt["trailerPositionX"];
            ego_state_msg.trailer_position_y = request_to_mqtt["trailerPositionY"];
            ego_state_msg.trailer_yaw = request_to_mqtt["trailerYaw"];
            ego_state_msg.gear_status = request_to_mqtt["gearStatus"];
            ego_state_msg.brake_percentage = request_to_mqtt["brakePercentage"];
            ego_state_msg.velocity = request_to_mqtt["velocity"];
            ego_state_msg.steering_angle = request_to_mqtt["steeringAngle"];
            ego_state_msg.throttle_percentage = request_to_mqtt["throttlePercentage"];
            ego_state_msg.localizationStatus = request_to_mqtt["localizationStatus"];
            ego_state_msg.head_lights = request_to_mqtt["headLights"];
            ego_state_msg.remote_emergency_button_status = request_to_mqtt["remoteEmergencyButtonStatus"];
            ego_state_msg.horn_cmd_feedback = request_to_mqtt["hornCmdFeedback"];
            ego_state_msg.apm_status = request_to_mqtt["apmStatus"];
            
            pub_to_ego_state_.publish(ego_state_msg);
          }
          catch (nlohmann::json::type_error& e) {
            std::cerr << "\n1-publishEgoStateMsgToROSTopic-Exception[type_error]: " << e.what() << std::endl;
          }
          catch (const std::exception& e) {
           std::cerr<<"\n2-publishEgoStateMsgToROSTopic-Exception[std]:" << e.what() << std::endl;
          }  
          catch (const std::logic_error& e) {
            std::cerr << "\n3-publishEgoStateMsgToROSTopic-Exception[logic_error]:" << e.what() << std::endl;
          }    
          catch (const nlohmann::json::parse_error& e) 
          {
            std::cerr << "\n4-publishEgoStateMsgToROSTopic-Exception[parse error]:"<< e.what() << std::endl;
          } 
        
          ////////////
          
          try
          {
            bool isResPub = false;
  
            if (((aios_session_job_status == 13) ||  (aios_session_job_status == 12)) 
                && 
                ((lane_type > 10 && lane_type <20) || (lane_type > 30 && lane_type <50))) //Working Lane
            {
              auto it = std::find(restricted_actions_.begin(), restricted_actions_.end(), 5);

              if (it != restricted_actions_.end()) 
              {    //found
                  restricted_actions_.erase(it);
                  isResPub = true;
              } 
                
            }
            else //restrict move by distance 
            {
              auto it = std::find(restricted_actions_.begin(), restricted_actions_.end(), 5);

              if (!(it != restricted_actions_.end())) 
              {    //not found
                  restricted_actions_.push_back(5);
                  isResPub = true;
              } 
            }
            if (isResPub)
            {
                  nlohmann::json msg_json;
                  msg_json["alertIds"] =  nlohmann::json::array();
                  msg_json["metricValues"]= nlohmann::json::array();
                  msg_json["restrictedActions"]= restricted_actions_;
                  msg_json["highRiskType"]= nlohmann::json::array();
                  msg_json["highRiskDist"]= nlohmann::json::array();
                  msg_json["jobDoneAllowed"] = true;
                  std_msgs::String msg;
                  msg.data =  msg_json.dump();
                  pub_to_remote_ops_request_.publish(msg);
              
            }
          }
          catch (nlohmann::json::type_error& e) {
            std::cerr << "\n1-publishRestrictedCommand-Exception[type_error]: " << e.what() << std::endl;
          }
          catch (const std::exception& e) {
           std::cerr<<"\n2-publishRestrictedCommand-Exception[std]:" << e.what() << std::endl;
          }  
          catch (const std::logic_error& e) {
            std::cerr << "\n3-publishRestrictedCommand-Exception[logic_error]:" << e.what() << std::endl;
          }    
          catch (const nlohmann::json::parse_error& e) 
          {
            std::cerr << "\n4-publishRestrictedCommand-Exception[parse error]:"<< e.what() << std::endl;
          } 

          //////////


        }

      
     
      }  
      loop_rate.sleep();
      }
      std::cout<<"[remote_handler] thread(sendEgoStateMsgToConsole) is shutting down..."<<std::endl;
    }
    catch (nlohmann::json::type_error& e) {
          std::cerr << "\n1-sendEgoStateMsgToConsole-Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
          std::cerr<<"\n2-sendEgoStateMsgToConsole-Exception[std]:" << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
          std::cerr << "\n3-sendEgoStateMsgToConsole-Exception[logic_error]:" << e.what() << std::endl;
    }    
    catch (const nlohmann::json::parse_error& e) 
    {
          std::cerr << "\n4-sendEgoStateMsgToConsole-Exception[parse error]:"<< e.what() << std::endl;
    } 
}

std::string RemoteHandler::getRemoteFrame(std::string aid_frame)
{
  if (aid_frame == aid_map_frame_)
  {
    return remote_map_frame_;
  }

  if (aid_frame == aid_odom_frame_)
  {
    return remote_odom_frame_;
  }

  if (aid_frame == aid_base_link_frame_)
  {
    return remote_base_link_frame_;
  }
  return "";
}

void RemoteHandler::sendTFMsgToConsole() {

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  
  nlohmann::json request_to_mqtt;
  request_to_mqtt["topicName"]    = "/remoteops/tf";
  request_to_mqtt["msgType"]      = "tf";
  
  ros::Rate loop_rate(tf_publish_rate_);
  
  while (ros::ok() && run_thread_tf_.load()) {
    
    if ( getLinkupStatus() ) {

      for (size_t i = 0 ; i < tf_frames_.size(); i++)
      {
          try {
           ROS_INFO_STREAM_ONCE("getting transform parent_frame_id: "<<tf_frames_[i]["frame_id"]<<" child_frame_id:"<<tf_frames_[i]["child_frame_id"]);
           transformStamped = tfBuffer.lookupTransform(tf_frames_[i]["frame_id"], tf_frames_[i]["child_frame_id"], ros::Time(0));  //the latest available transform in the buffer
           bool error = false;
           if (getRemoteFrame(transformStamped.header.frame_id) == "")
           {
             ROS_ERROR_STREAM("1-sendTFMsgToConsole-getRemoteFrame(frame_id)== null");
             error = true;
           }
           if (getRemoteFrame(transformStamped.child_frame_id) == "")
           {
             ROS_ERROR_STREAM("1-sendTFMsgToConsole-getRemoteFrame(child_frame_id)== null");
             error = true;
           }
           if (error)
           {
            continue;
           }
           if (transformStamped.header.stamp.toSec() == 0)
           {
            transformStamped.header.stamp = ros::Time::now();
           }
          
           request_to_mqtt["stamp"]        = getRosTimeNanosec(transformStamped.header);
           request_to_mqtt["frameId"]      = getRemoteFrame(transformStamped.header.frame_id);
           request_to_mqtt["childFrameId"] = getRemoteFrame(transformStamped.child_frame_id);
           
           request_to_mqtt["translationX"] = transformStamped.transform.translation.x;  
           request_to_mqtt["translationY"] = transformStamped.transform.translation.y; 
           request_to_mqtt["translationZ"] = transformStamped.transform.translation.z; 

           request_to_mqtt["rotationX"] = transformStamped.transform.rotation.x;  
           request_to_mqtt["rotationY"] = transformStamped.transform.rotation.y;  
           request_to_mqtt["rotationZ"] = transformStamped.transform.rotation.z;  
           request_to_mqtt["rotationW"] = transformStamped.transform.rotation.w;  
     
           publishToMqtt(request_to_mqtt);

          } 
          catch (tf2::TransformException& ex) 
          {
            ROS_WARN_STREAM_ONCE("tf2::TransformException: "<<ex.what());
          }
          catch (nlohmann::json::type_error& e) {
            std::cerr << "\n1-sendTFMsgToConsole-Exception[type_error]:" << e.what() << std::endl;
          }
          catch (const std::exception& e) {
            std::cerr << "\n2-sendTFMsgToConsole-Exception[std]:" << e.what() << std::endl;
          } 
          catch (const std::logic_error& e) {
           std::cerr << "\n3-sendTFMsgToConsole-Exception[logic_error]:" << e.what() << std::endl;
          }    
      }
     
    }
    loop_rate.sleep();
  }
  std::cout<<"[remote_handler] thread(sendTFMsgToConsole) is shutting down..."<<std::endl;
}
  
void RemoteHandler::publishRemoteOpsRequest()
{
  if ( getLinkupStatus() ) {
   //Added to enable Job Done button
    nlohmann::json msg_json;
    msg_json["alertIds"] =  nlohmann::json::array();
    msg_json["metricValues"]= nlohmann::json::array();
    msg_json["restrictedActions"]= nlohmann::json::array();
    msg_json["highRiskType"]= nlohmann::json::array();
    msg_json["highRiskDist"]= nlohmann::json::array();
    msg_json["jobDoneAllowed"] = true;
    std_msgs::String msg;
    msg.data =  msg_json.dump();
    pub_to_remote_ops_request_.publish(msg);
  }
}

geometry_msgs::PoseStamped  RemoteHandler::transformPose(const geometry_msgs::PoseStamped& input_pose, const std::string& target_frame)
{
    // Create a TF2 buffer and listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
   
   // Transform the input pose
    geometry_msgs::PoseStamped transformed_pose;
     
    try
    {
        geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(target_frame, input_pose.header.frame_id, ros::Time(0));

        tf2::doTransform(input_pose, transformed_pose, transform_stamped);

         
       ROS_WARN_STREAM("Ego state: vehX:"<<transformed_pose.pose.position.x<<" vehY:"<<transformed_pose.pose.position.y);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("transformPose error: %s", ex.what());
    }
    return transformed_pose;
   
}

geometry_msgs::Quaternion RemoteHandler::yawToQuaternion(double yaw)
{
    // Create a quaternion object
    tf2::Quaternion q;

    // Set the quaternion from roll, pitch, and yaw (only yaw in this case)
    q.setRPY(0, 0, yaw);

    // Convert to geometry_msgs::Quaternion
    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();

    return quat_msg;
}

double RemoteHandler::quaternionToYaw(const geometry_msgs::Quaternion& quaternion)
{
    // Convert geometry_msgs::Quaternion to tf2::Quaternion
    tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

    // Convert tf2::Quaternion to a 3x3 rotation matrix
    tf2::Matrix3x3 rotation_matrix(tf_quaternion);

    // Extract roll, pitch, and yaw from the rotation matrix
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    return yaw; // Return only the yaw angle
}
  
  
  
  /*****************************************************************************************************
 * Destructor
 *****************************************************************************************************/
RemoteHandler::~RemoteHandler() {}

/*****************************************************************************************************
*****************************************************************************************************/
int main(int argc, char** argv) {
  ros::init(argc, argv, "remote_handler");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ROS_WARN_STREAM("Initializing Remote Handler");

  int count = 0;
  RemoteHandler remote_handler_obj(nh, private_nh);

  double ping_freq = remote_handler_obj.getPingFrequency();
  double ping_timeout_ms = remote_handler_obj.getPingTimeout() * 1000;
  double ping_period_ms = (1 / ping_freq) * 1000;

  std::thread t_send_ego_state([&]() { remote_handler_obj.sendEgoStateMsgToConsole(); });
  std::thread t_send_tf([&]() { remote_handler_obj.sendTFMsgToConsole(); });

  double main_loop_rate = remote_handler_obj.getMainLoopRate();
  ros::Rate r(main_loop_rate);
  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();               // Start the spinner

  while (ros::ok()) {
    if (remote_handler_obj.getVehicleID() == "") {
      ROS_ERROR_STREAM("Preprocess/ APM ID not initialized - retrying in 1 second");
      sleep(1);
      continue;
    }

    else if (!remote_handler_obj.isBrokerConnected()) {
      ROS_ERROR_STREAM("Broker not connected - retrying every 1 second");
      remote_handler_obj.initializeMqttComms();
      sleep(1);
      continue;
    }

    if (remote_handler_obj.getLinkupStatus()) {
      uint64_t curr_ts = remote_handler_obj.getTimestampNanosec();
      auto time_diff = (curr_ts - remote_handler_obj.getLastPingMsgTimestamp());
      std::chrono::nanoseconds duration_ns(time_diff);
      std::chrono::milliseconds ping_duration_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(duration_ns);
      bool isBreaklink = false;

      if (remote_handler_obj.isBreaklinkEnabled()) {
        // calculate timeout
        if (!remote_handler_obj.isFirstPing()) {
          bool ping_ack_status = remote_handler_obj.getPingAckStatus();

          if (ping_ack_status) {
            uint64_t request_latency = remote_handler_obj.getPingRequestLatency();
            uint64_t response_latency = remote_handler_obj.getPingResponseLatency();
            uint64_t abnormal_val = 18446744073709500000;

            if (request_latency >= abnormal_val) {
              ROS_WARN_STREAM("Ignoring request Latency abnormal cond:" << request_latency);
              request_latency = 0;
            }

            if (response_latency >= abnormal_val) {
              ROS_WARN_STREAM("Ignoring response_latency abnormal cond:" << response_latency);
              response_latency = 0;
            }

            auto total_latency = request_latency + response_latency;

            if (total_latency > ping_timeout_ms) {
              ROS_ERROR_STREAM("1-VEHICLE BREAKLINK(TIMEOUT) !! requestLatency:"<< request_latency << " responseLatency:" << response_latency);
              // send breaklink request
              remote_handler_obj.sendRemoteRequest("breaklink");
              isBreaklink = true;
            }
          }
          // no console ack received
          else {
            uint64_t last_console_ack_sent_ts = remote_handler_obj.getLastConsoleAckSentTimestamp();

            if (last_console_ack_sent_ts != 0) {
              auto ack_time_diff = (curr_ts - last_console_ack_sent_ts);
              std::chrono::nanoseconds ack_duration_ns(ack_time_diff);
              std::chrono::milliseconds ack_ping_duration_ms =
                  std::chrono::duration_cast<std::chrono::milliseconds>(ack_duration_ns);

              if (ack_ping_duration_ms.count() > ping_timeout_ms) {
                ROS_WARN_STREAM("2- BREAKLINK!!!NO ACK RECEIVED FOR milliSec: "
                                << ack_ping_duration_ms.count());
                // send breaklink request
                remote_handler_obj.sendRemoteRequest("breaklink");
                isBreaklink = true;
              }
            }
          }
        }
      }
      if (((remote_handler_obj.isFirstPing()) || (ping_duration_ms.count() >= ping_period_ms)) &&
          !isBreaklink) {
        if (remote_handler_obj.isSendPingRequest()) {
          remote_handler_obj.sendRemoteRequest("ping");
        }
      }
    }
    else 
    {
      ROS_ERROR_THROTTLE(1.0, "NOT LINKED UP!!");
    }
    
    r.sleep();
  }

  spinner.stop();
  remote_handler_obj.run_thread_tf_.store(false);
  remote_handler_obj.run_thread_ego_.store(false);
  if (t_send_tf.joinable())
  {
    t_send_tf.join();
  }
  if (t_send_ego_state.joinable())
  {
    t_send_ego_state.join();
  }
 
  return 0;
}
