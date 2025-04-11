/*******************************************************
 *
 *    _    ___  ___   ___  ___ __   __ ___  ___  ___
 *   /_\  |_ _||   \ | _ \|_ _|\ \ / /| __|| _ \/ __|
 *  / _ \  | | | |) ||   / | |  \ V / | _| |   /\__ \
 * /_/ \_\|___||___/ |_|_\|___|  \_/  |___||_|_\|___/
 *
 *
 * Copyright (C) 2022 AIDE @ AIDRIVERS Ltd - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * author = 'Zeynep'
 * email  = 'zeynep@aidrivers.ai'
 *
 *******************************************************/
#include <remote_ops/remote_ops.h>

RemoteOps::RemoteOps(ros::NodeHandle& private_nh, std::shared_ptr<mqtt::async_client> cli)
    : private_nh_(private_nh), cli_(cli), vehicle_control_mode_(-1), avcs_job_status_(-1), apm_status_(-1), session_job_status_(0) {
  user_id_ = std::getenv("USER");
  log_folder_ = "/home/" + user_id_ + "/remote_ops_logs/" + currentDateTime() + "/";
  private_nh_.param("mqtt_pub_wait_sec", mqtt_pub_wait_sec_, 0.0);
  pub_outgoing_payload_ = private_nh_.advertise<std_msgs::String>("/remoteops/outgoing_payload", 1);

  vec_visualization_topic_.push_back("/remoteops/ego_state");
  vec_visualization_topic_.push_back("/remoteops/vehicle_control_mode");
  vec_visualization_topic_.push_back("/remoteops/avcs_job_status");
  vec_visualization_topic_.push_back("/remoteops/ehmi_info");
  vec_visualization_topic_.push_back("/remoteops/avcs_destination_from_ui");
  vec_visualization_topic_.push_back("/remoteops/traffic_light_status");
  vec_visualization_topic_.push_back("/remoteops/ops_response");
  vec_visualization_topic_.push_back("/remoteops/tf");
  vec_visualization_topic_.push_back("/remoteops/predicted_footprint");
  vec_visualization_topic_.push_back("/remoteops/route_plan");
  //PCDs
  vec_visualization_topic_.push_back("/remoteops/vertical_points"); 
  vec_visualization_topic_.push_back("/remoteops/on_road_pc"); 
  vec_visualization_topic_.push_back("/remoteops/curb_pc"); 
  vec_visualization_topic_.push_back("/remoteops/most_constrained_points"); 
  vec_visualization_topic_.push_back("/remoteops/road_intensity_detection"); 
  
  //MARKERS
  vec_visualization_topic_.push_back("/remoteops/most_constrained_object"); 
  vec_visualization_topic_.push_back("/remoteops/objects"); 
  vec_visualization_topic_.push_back("/remoteops/crosswalk_vis"); 
  vec_visualization_topic_.push_back("/remoteops/traffic_jam_lanes_vis");
  vec_visualization_topic_.push_back("/remoteops/road_global_vis");
  vec_visualization_topic_.push_back("/remoteops/objects_of_interest");
  vec_visualization_topic_.push_back("/remoteops/hatch_cover_detection_box");
  vec_visualization_topic_.push_back("/remoteops/rtg_detection_box");


  //APM->Console
  vec_visualization_topic_.push_back("/remoteops/remote_ops_request"); 
  vec_visualization_topic_.push_back("/remoteops/remote_ops_init"); 
}

// Function to get current time in readable format
std::string RemoteOps::currentDateTime() {
  std::string current_time;
  ros::Time current_ros_time = ros::Time::now();
  double unix_time = current_ros_time.toSec();

  std::time_t unix_time_t = static_cast<std::time_t>(unix_time);

  // Convert std::time_t to std::tm
  std::tm* timeinfo = std::localtime(&unix_time_t);

  char buffer[80];
  std::strftime(buffer, 80, "%Y-%m-%d-%H-%M-%s", timeinfo);
  current_time = buffer;
  return current_time;
}

void RemoteOps::ROS_INFO_STREAM_COLOR(const std::string& color, auto text) {
  std::unordered_map<std::string, std::string> colorMap{
      {"black", "0;30"}, {"red", "0;31"},     {"green", "0;32"}, {"yellow", "0;33"},
      {"blue", "0;34"},  {"magenta", "0;35"}, {"cyan", "0;36"},  {"white", "0;37"}};

  auto it = colorMap.find(color);
  if (it != colorMap.end()) {
    ROS_INFO_STREAM("\033[" << it->second << "m" << text << "\033[0m");
  } else {
    ROS_INFO_STREAM(text);
  }
}

uint64_t RemoteOps::getTimestamp() {
  const auto now = std::chrono::system_clock::now();
  // const auto epoch = now.time_since_epoch();

  // Convert the time point to time since epoch in nanoseconds
  auto epoch_time_ns =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();

  //  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
  return epoch_time_ns;  // seconds.count();
}

uint32_t RemoteOps::CRC32(const std::string& in) {
  boost::crc_32_type result;
  result.process_bytes(in.data(), in.length());
  return result.checksum();
}

void RemoteOps::setBreaklinkReason(std::string breaklink_reason) {
  if (breaklink_reason == "notApplicable") {
    breaklink_reason_ = BreakLinkReason::NOT_APPLICABLE;
  } else if (breaklink_reason == "timeout") {
    breaklink_reason_ = BreakLinkReason::TIMEOUT;
  } else if (breaklink_reason == "hardwareDisconnect") {
    breaklink_reason_ = BreakLinkReason::HARDWARE_DISCONNECT;
  } else if (breaklink_reason == "teardownTimeout") {
    breaklink_reason_ = BreakLinkReason::TEARDOWN_TIMEOUT;
  } else if (breaklink_reason == "maxLinkupAttempts") {
    breaklink_reason_ = BreakLinkReason::MAX_LINKUP_ATTEMPTS;
  } else if (breaklink_reason == "webuiHeartbeatLost") {
    breaklink_reason_ = BreakLinkReason::WEBUI_HEARTBEAT_LOST;
  }
}

std::string RemoteOps::getBreaklinkReason(BreakLinkReason breaklink_reason) {
  switch (breaklink_reason) {
    case BreakLinkReason::NOT_APPLICABLE:
      return "notApplicable";
    case BreakLinkReason::TIMEOUT:
      return "timeout";
    case BreakLinkReason::HARDWARE_DISCONNECT:
      return "hardwareDisconnect";
    case BreakLinkReason::TEARDOWN_TIMEOUT:
      return "teardownTimeout";
    case BreakLinkReason::MAX_LINKUP_ATTEMPTS:
      return "maxLinkupAttempts";
    case BreakLinkReason::WEBUI_HEARTBEAT_LOST:
      return "webuiHeartbeatLost";
    default:
      break;
  }

  return "";
};

std::string RemoteOps::getTeardownResult(TeardownResult result) {
  switch (result) {
    case TeardownResult::NOT_APPLICABLE:
      return "notApplicable";
    case TeardownResult::SUCCESS:
      return "success";
    case TeardownResult::FAIL:
      return "fail";
    default:
      break;
  }

  return "";
}

std::string RemoteOps::getLinkupResult(LinkUpResult result) {
  switch (result) {
    case LinkUpResult::NOT_APPLICABLE:
      return "notApplicable";
    case LinkUpResult::SUCCESS:
      return "success";
    case LinkUpResult::FAIL_NOT_IN_VALID_STATE:
      return "failNotInValidState";
    case LinkUpResult::FAIL_ALREADY_CONTROLLED:
      return "failAlreadyControlled";
    case LinkUpResult::FAIL_NOT_EXPECTING_LINKUP:
      return "failNotExpectingLinkup";
    default:
      break;
  }

  return "";
}

uint8_t RemoteOps::getOperationType(std::string msg_type)
{
  if (msg_type == "stop") 
  {
    return 0;
  }
  else if (msg_type == "goal") 
  {
    return 1;
  }
  else if (msg_type == "location") 
  {
    return 2;
  }
  else if (msg_type == "traffic_light_override") 
  {
    return 3;
  }
  else if (msg_type == "manual_push_remote") 
  {
    return 4;
  }
  else if (msg_type == "adjust_position") 
  {
    return 5;
  }
  else if (msg_type == "cmd_horn") 
  {
    return 6;
  }
  else if (msg_type == "cmd_signal_light") 
  {
    return 7;
  }
  else if (msg_type == "emergency_brake_command") 
  {
    return 8;
  }
  else if (msg_type == "override_path") 
  {
    return 10;
  }
  else if (msg_type == "non_yard_goal") 
  {
    return 12;
  }
  else if (msg_type == "cmd_head_light_remote") 
  {
    return 13;
  }
  else if (msg_type == "precedence_override") 
  {
    return 14;
  }

 return 99;
}


/************************************************************************************************************
 * Serialize command messages from Remote Console to send to Vehicle
 ************************************************************************************************************/
void RemoteOps::serializeMsgFromConsole(std::string topic_name, std::string msg_type, bool isEmergencyBrake, bool isHazardLights,
                                        uint8_t nudgeInstanceId) {
  int qos = C_QOS;
  ::capnp::MallocMessageBuilder msg;

  std::size_t found = topic_name.rfind('/');
  if (found != std::string::npos) {
    msg_type = topic_name.substr(found + 1);
  }
  if (msg_type == "linkup") {
    LinkUp::Builder link_up_builder = msg.initRoot<LinkUp>();
    ::capnp::MallocMessageBuilder link_up_from_console_msg;
    LinkUpFromConsole::Builder link_up_from_console_builder =
        link_up_from_console_msg.initRoot<LinkUpFromConsole>();
    link_up_from_console_builder.setConsoleId(console_id_);
    link_up_from_console_builder.setVehicleId(vehicle_id_);
    link_up_from_console_builder.setViewType(view_type_);
    link_up_from_console_builder.setConsoleMode(console_mode_);
    link_up_builder.getCommand().setLinkUpFromConsole(link_up_from_console_builder);
  } else if (msg_type == "breaklink") {
    BreakLink::Builder breaklink_builder = msg.initRoot<BreakLink>();
    ::capnp::MallocMessageBuilder breaklink_from_console_msg;
    BreakLinkFromConsole::Builder breaklink_from_console_builder =
        breaklink_from_console_msg.initRoot<BreakLinkFromConsole>();
    breaklink_from_console_builder.setConsoleId(console_id_);
    breaklink_from_console_builder.setVehicleId(vehicle_id_);
    breaklink_from_console_builder.setBreakLinkReason(breaklink_reason_);
    breaklink_builder.getCommand().setBreakLinkFromConsole(breaklink_from_console_builder);
  } else if (msg_type == "teardown") {
    Teardown::Builder teardown_builder = msg.initRoot<Teardown>();
    ::capnp::MallocMessageBuilder teardown_from_console_msg;
    TeardownFromConsole::Builder teardown_from_console_builder =
        teardown_from_console_msg.initRoot<TeardownFromConsole>();
    teardown_from_console_builder.setConsoleId(console_id_);
    teardown_from_console_builder.setVehicleId(vehicle_id_);
    teardown_builder.getCommand().setTeardownFromConsole(teardown_from_console_builder);
  } else if (msg_type == "ping") {
    qos = V_QOS;
    // Console ping request
    PingMessage::Builder ping_builder = msg.initRoot<PingMessage>();
    ::capnp::MallocMessageBuilder ping_from_console_msg;
    Ping::Builder ping_from_console_builder = ping_from_console_msg.initRoot<Ping>();
    ping_builder.setSender(console_id_);
    ping_builder.setReceiver(vehicle_id_);
    ping_builder.setSeq(console_send_ping_seq_);
    ping_builder.getType().setPing(ping_from_console_builder);
    console_send_ping_seq_++;
  } else if (msg_type == "emergency_brake_command") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder emergency_brake_msg;
    EmergencyBrakeCmd::Builder emergency_brake_builder =
        emergency_brake_msg.initRoot<EmergencyBrakeCmd>();
    emergency_brake_builder.setApplyBrake(isEmergencyBrake);
    remoteops_cmd_builder.getCommand().setEmergencyBrakeCmd(emergency_brake_builder);
  } else if (msg_type == "cmd_signal_light") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder signal_light_msg;
    HazardLightCmd::Builder signal_light_builder = signal_light_msg.initRoot<HazardLightCmd>();
    if (isHazardLights)
    {
      signal_light_builder.setState(3);
    }
    else 
    {
       signal_light_builder.setState(0);
    }
    remoteops_cmd_builder.getCommand().setHazardLightCmd(signal_light_builder);
  } else if (msg_type == "cmd_head_light_remote") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder head_light_msg;
    HeadLightCmd::Builder head_light_remote_builder = head_light_msg.initRoot<HeadLightCmd>();
    head_light_remote_builder.setState(
        head_light_state_);  // [0 - Head light off], [1 - Low beam], [2 - High beam]
    remoteops_cmd_builder.getCommand().setHeadLightCmd(head_light_remote_builder);
  } else if (msg_type == "cmd_horn") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder horn_msg;
    HornCmd::Builder horn_builder = horn_msg.initRoot<HornCmd>();
    horn_builder.setActivateHorn(true);
    remoteops_cmd_builder.getCommand().setHornCmd(horn_builder);
  } /*else if (msg_type == "cmd_complete_job") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder complete_job_msg;
    CompleteJobCmd::Builder complete_job_builder = complete_job_msg.initRoot<CompleteJobCmd>();
    complete_job_builder.setState(true);
    remoteops_cmd_builder.getCommand().setCompleteJobCmd(complete_job_builder);
  }*/
  else if (msg_type == "stop") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder stop_msg;
    PauseResumeCmd::Builder stop_builder = stop_msg.initRoot<PauseResumeCmd>();
    stop_builder.setPause(pause_);
    remoteops_cmd_builder.getCommand().setPauseResumeCmd(stop_builder);
  } else if (msg_type == "traffic_light_override") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder tl_override_msg;
    TrafficLightOverrideCmd::Builder tl_override_builder =
        tl_override_msg.initRoot<TrafficLightOverrideCmd>();
    tl_override_builder.setType(static_cast<uint8_t>(traffic_light_override_state_));
    remoteops_cmd_builder.getCommand().setTrafficLightOverrideCmd(tl_override_builder);
  } else if (msg_type == "precedence_override") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder precedence_override_msg;
    PrecedenceOverrideCmd::Builder precedence_override_builder =
        precedence_override_msg.initRoot<PrecedenceOverrideCmd>();
    precedence_override_builder.setGo(precedence_override_state_);
    remoteops_cmd_builder.getCommand().setPrecedenceOverrideCmd(precedence_override_builder);
  } else if (msg_type == "goal") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder goal_msg;
    SetDestinationCmd::Builder goal_builder = goal_msg.initRoot<SetDestinationCmd>();
    // dummy values/remote_console/block_id
    std::string block_name;
    int block_id, lane_id, slot_id, container_type, job_type;
    private_nh_.param("/remote_console/block_name", block_name, std::string("TD"));
    private_nh_.param("/remote_console/block_id", block_id, 2);
    private_nh_.param("/remote_console/lane_id", lane_id, 11);
    private_nh_.param("/remote_console/slot_id", slot_id, 28);
    private_nh_.param("/remote_console/container_type", container_type, 1);
    private_nh_.param("/remote_console/job_type", job_type, 1);

    goal_builder.setBlockName(block_name);
    goal_builder.setBlockId(static_cast<uint8_t>(block_id));
    goal_builder.setLaneId(static_cast<uint8_t>(lane_id));
    goal_builder.setSlotId(static_cast<uint8_t>(slot_id));
    goal_builder.setContainerType(static_cast<uint8_t>(container_type));
    goal_builder.setJobType(static_cast<uint8_t>(job_type));
    remoteops_cmd_builder.getCommand().setSetDestinationCmd(goal_builder);
  } else if (msg_type == "adjust_position") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder adjust_position_msg;
    AdjustPositionByDistanceCmd::Builder adjust_position_builder =
        adjust_position_msg.initRoot<AdjustPositionByDistanceCmd>();
    // dummy values
    double distance;
    private_nh_.param("/remote_console/move_by_distance", distance, 2.0);

    bool safety_override = true;
    adjust_position_builder.getCommandType().setDistance(distance);
    // adjust_position_builder.getCommandType().setSafetyOverride(safety_override);
    remoteops_cmd_builder.getCommand().setAdjustPositionByDistanceCmd(adjust_position_builder);
  } else if (msg_type == "non_yard_goal") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder nonyard_goal_msg;
    SetNonYardDestinationCmd::Builder nonyard_goal_builder =
        nonyard_goal_msg.initRoot<SetNonYardDestinationCmd>();
    // dummy values
    int destination_id;
    private_nh_.param("/remote_console/destination_id", destination_id, 18);

    nonyard_goal_builder.setDestination(static_cast<uint32_t>(destination_id));
    // adjust_position_builder.getCommandType().setSafetyOverride(safety_override);
    remoteops_cmd_builder.getCommand().setSetNonYardDestinationCmd(nonyard_goal_builder);
  } else if (msg_type == "location") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder location_msg;
    RelocalizationCmd::Builder location_builder = location_msg.initRoot<RelocalizationCmd>();
    // dummy values
    double x = 20.0;
    double y = 30.0;
    double yaw = 10.0;

    location_builder.setX(x);
    location_builder.setY(y);
    location_builder.setYaw(yaw);
    // adjust_position_builder.getCommandType().setSafetyOverride(safety_override);
    remoteops_cmd_builder.getCommand().setRelocalizationCmd(location_builder);
  } else if (msg_type == "manual_push_remote") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder manual_push_remote_msg;
    NudgeForwardCmd::Builder manual_push_remote_builder =
        manual_push_remote_msg.initRoot<NudgeForwardCmd>();

    uint8_t engage = 1;
    uint8_t nudge_instance_id = nudgeInstanceId;
    manual_push_remote_builder.setEngage(engage);
    manual_push_remote_builder.setNudgeInstanceId(nudge_instance_id);
    remoteops_cmd_builder.getCommand().setNudgeForwardCmd(manual_push_remote_builder);
  } else if (msg_type == "override_path") {
    RemoteOpsCommand::Builder remoteops_cmd_builder = msg.initRoot<RemoteOpsCommand>();
    ::capnp::MallocMessageBuilder override_path_msg;
    TrajectoryOverrideCmd::Builder override_path_builder =
        override_path_msg.initRoot<TrajectoryOverrideCmd>();

    ::capnp::MallocMessageBuilder path_msg;
    Path::Builder path_builder = path_msg.initRoot<Path>();

    uint64_t timestamp = getTimestamp();
    path_builder.setTimestamp(timestamp);

    // Initialize the list with a length of 5
    ::capnp::List<uint32_t>::Builder segment_list = path_builder.initSegmentId(5);
    
    ::capnp::List<float>::Builder x_list = path_builder.initDiscretizedPathPointsX(5);
    ::capnp::List<float>::Builder y_list = path_builder.initDiscretizedPathPointsY(5);
    ::capnp::List<float>::Builder yaw_list = path_builder.initDiscretizedPathPointsYaw(5);

    // Write values to the list
    for (int i = 0; i < 5; ++i) {
      segment_list.set(i, i);
      x_list.set(i, i * 10);  // Set each element to i * 10
      y_list.set(i, i * 10 + 1);
      yaw_list.set(i, i / 2);
    }

    // path_builder.setSegmentId(segment_list);
    // path_builder.setDiscretizedPathPointsX(x_list);
    // path_builder.setDiscretizedPathPointsY(y_list);
    // path_builder.setDiscretizedPathPointsYaw(yaw_list);

    override_path_builder.setPath(path_builder);
    remoteops_cmd_builder.getCommand().setTrajectoryOverrideCmd(override_path_builder);
  }
  uint64_t msg_ts;
  bool errorRaised = false;
  auto serialized_message = encodeMessage(topic_name, msg, msg_ts, errorRaised);
  if (errorRaised)
  {
    return;
  }

  if (qos == C_QOS) {
    // CheckSum for QOS=2 messages
    auto crc32 = CRC32(serialized_message);
    serialized_message.append(reinterpret_cast<const char*>(&crc32), sizeof(crc32));
  }

  publishMQTTMessage(msg_type, serialized_message, topic_name, qos, "console");

  if (!debug_ && msg_type == "ping") {
    return;
  }
  std::string message_out = "Serializing [" + topic_name + "] from CONSOLE: ";
  ROS_INFO_STREAM_COLOR(CONSOLE_MSG_COLOR, message_out);
};

/************************************************************************************************************
 * Serialize visualizations messages from Vehicle to send to Remote Console
 ************************************************************************************************************/
void RemoteOps::serializeMsgFromVehicle(nlohmann::json& payload, bool& errorRaised) {

    ::capnp::MallocMessageBuilder msg;

    int qos = 0;
    bool isVehiclePingReq = false;
    bool isVehiclePingAck = false;
    std::string topic_name = payload["topicName"];
    std::string msg_type = payload["msgType"];
  try
  {
    if (msg_type == "linkup") {
      try 
      {
          qos = C_QOS;

          LinkUp::Builder link_up_builder = msg.initRoot<LinkUp>();

          ::capnp::MallocMessageBuilder link_up_from_console_ack_msg;

          LinkUpFromConsoleAck::Builder link_up_from_console_ack_builder =
              link_up_from_console_ack_msg.initRoot<LinkUpFromConsoleAck>();
          std::string console_id = payload["consoleID"];
          std::string vehicle_id = payload["vehicleID"];
          ConsoleViewType view_type = payload["viewType"];
          LinkUpResult result = payload["result"];

          link_up_from_console_ack_builder.setConsoleId(console_id);
          link_up_from_console_ack_builder.setVehicleId(vehicle_id);
          link_up_from_console_ack_builder.setViewType(view_type);
          link_up_from_console_ack_builder.setResult(result);
          link_up_builder.getCommand().setLinkUpFromConsoleAck(link_up_from_console_ack_builder);
      }  
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - linkup [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  linkup [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }  
      catch (nlohmann::json::type_error& e) {
        std::cerr << "\n3-ERROR-linkup [serializeMsgFromVehicle][type_error]:" << e.what() << std::endl;
        errorRaised = true; 
        return;
      }
      catch (const nlohmann::json::parse_error& e) 
      {
        std::cerr << "\n4-ERROR-linkup [serializeMsgFromVehicle][parse_error]:"<< e.what() << std::endl;
        errorRaised = true; 
        return;
      }  
      catch (const std::logic_error& e) {
        std::cerr << "\n5-ERROR-linkup [serializeMsgFromVehicle][logic_error]:"<< e.what() << std::endl;
        errorRaised = true; 
        return;
      } 
    } else if (msg_type == "breaklink") {
      try
      {
          qos = C_QOS;
          BreakLink::Builder breaklink_builder = msg.initRoot<BreakLink>();
          std::string console_id = payload["consoleID"];
          std::string vehicle_id = payload["vehicleID"];
          if (payload.contains("breakLinkReason")) {
            // vehicle is sending breaklink request
            ROS_WARN("****Vehicle is sending breaklink request******");
            ::capnp::MallocMessageBuilder breaklink_from_console_msg;
            BreakLinkFromConsole::Builder breaklink_from_console_builder =
                breaklink_from_console_msg.initRoot<BreakLinkFromConsole>();
            BreakLinkReason break_link_reason = payload["breakLinkReason"];
            breaklink_from_console_builder.setConsoleId(console_id);
            breaklink_from_console_builder.setVehicleId(vehicle_id);
            breaklink_from_console_builder.setBreakLinkReason(break_link_reason);
            breaklink_builder.getCommand().setBreakLinkFromConsole(breaklink_from_console_builder);
            isVehicleBreakLinkRequest_ = true;
          } else if (!isVehicleBreakLinkRequest_) {
            // sending breaklink ack

            ::capnp::MallocMessageBuilder breaklink_from_console_ack_msg;
            BreakLinkFromConsoleAck::Builder breaklink_from_console_ack_builder =
                breaklink_from_console_ack_msg.initRoot<BreakLinkFromConsoleAck>();
            breaklink_from_console_ack_builder.setConsoleId(console_id);
            breaklink_from_console_ack_builder.setVehicleId(vehicle_id);

            breaklink_builder.getCommand().setBreakLinkFromConsoleAck(breaklink_from_console_ack_builder);
          }
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - breaklink [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR - breaklink [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (nlohmann::json::type_error& e) {
        std::cerr << "\n3-ERROR - breaklink [serializeMsgFromVehicle][type_error]: " << e.what() << std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::logic_error& e) {
        std::cerr << "\n4-ERROR - breaklink [serializeMsgFromVehicle][logic_error]: " << e.what() << std::endl;
        errorRaised = true; 
        return;
      } 

    } else if (msg_type == "consoleBreaklinkAck") {
      try
      {
          std::string msg_out = "Serializing CONSOLE BREAKLINK ACK";
          ROS_INFO_STREAM_COLOR("cyan", msg_out);
          qos = C_QOS;
          BreakLink::Builder breaklink_builder = msg.initRoot<BreakLink>();
          std::string console_id = payload["consoleID"];
          std::string vehicle_id = payload["vehicleID"];

          ::capnp::MallocMessageBuilder breaklink_from_console_ack_msg;
          BreakLinkFromConsoleAck::Builder breaklink_from_console_ack_builder =
              breaklink_from_console_ack_msg.initRoot<BreakLinkFromConsoleAck>();
          breaklink_from_console_ack_builder.setConsoleId(console_id);
          breaklink_from_console_ack_builder.setVehicleId(vehicle_id);
          breaklink_builder.getCommand().setBreakLinkFromConsoleAck(breaklink_from_console_ack_builder);
          msg_out = "CONSOLE Sending BREAKLINK ACK: " + payload.dump();
          ROS_INFO_STREAM_COLOR("cyan", msg_out);
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - consoleBreaklinkAck [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  consoleBreaklinkAck [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  consoleBreaklinkAck [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  consoleBreaklinkAck [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 

    } else if (msg_type == "teardown") {
      try
      {
        qos = C_QOS;
        Teardown::Builder teardown_builder = msg.initRoot<Teardown>();

        ::capnp::MallocMessageBuilder teardown_from_console_ack_msg;
        TeardownFromConsoleAck::Builder teardown_from_console_ack_builder =
            teardown_from_console_ack_msg.initRoot<TeardownFromConsoleAck>();

        std::string console_id = payload["consoleID"];
        std::string vehicle_id = payload["vehicleID"];
        TeardownResult result = payload["result"];
        teardown_from_console_ack_builder.setConsoleId(console_id);
        teardown_from_console_ack_builder.setVehicleId(vehicle_id);
        teardown_from_console_ack_builder.setResult(result);
        teardown_builder.getCommand().setTeardownFromConsoleAck(teardown_from_console_ack_builder);
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - teardown [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  teardown [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  teardown [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  teardown [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
    } else if (msg_type == "ping") {
      try
      {
          qos = V_QOS;  // QOS = 0
          PingMessage::Builder ping_msg_builder = msg.initRoot<PingMessage>();
          std::string sender = payload["sender"];
          std::string receiver = payload["receiver"];
          auto seq = payload["seq"];
          ping_msg_builder.setSender(sender);
          ping_msg_builder.setReceiver(receiver);
          ping_msg_builder.setSeq(seq);

          if (payload.contains("requestLatency"))  // vehicle sending ping ACK
          {
            ::capnp::MallocMessageBuilder ping_ack_msg;
            PingAck::Builder ping_ack_builder = ping_ack_msg.initRoot<PingAck>();
            uint64_t latency = vehicle_ack_ping_request_latency_;
            ping_ack_builder.setRequestLatency(latency);
            ping_msg_builder.getType().setPingAck(ping_ack_builder);
            //  ROS_INFO_STREAM("vehicle ping ack LATENCY(ms):"<< latency);
            isVehiclePingAck = true;
            //  if (debug_)
            //  {
            //    std::string msg_out = "VEHICLE Sending PING ACK: "+payload.dump();
            //    ROS_INFO_STREAM_COLOR("magenta", msg_out);
            //  }
          } else  // vehicle sending ping REQUEST
          {
            ::capnp::MallocMessageBuilder ping_msg;
            Ping::Builder ping_builder = ping_msg.initRoot<Ping>();
            ping_msg_builder.getType().setPing(ping_builder);
            isVehiclePingReq = true;
          }
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - ping [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  ping [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  ping [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  ping [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 

    } else if (msg_type == "consolePingAck")  // Console Ping Ack message To Vehicle
    {
      try
      {
          if (debug_) {
            std::string msg_out = "Serializing CONSOLE PINGACK";
            ROS_INFO_STREAM_COLOR("cyan", msg_out);
          }

          qos = V_QOS;
          PingMessage::Builder ping_msg_builder = msg.initRoot<PingMessage>();
          std::string sender = payload["sender"];
          std::string receiver = payload["receiver"];
          uint8_t seq = payload["seq"];
          uint64_t requestLatency = payload["requestLatency"];
          ping_msg_builder.setSender(sender);
          ping_msg_builder.setReceiver(receiver);
          ping_msg_builder.setSeq(seq);
          ::capnp::MallocMessageBuilder ping_ack_msg;
          PingAck::Builder ping_ack_builder = ping_ack_msg.initRoot<PingAck>();
          ping_ack_builder.setRequestLatency(requestLatency);
          ping_msg_builder.getType().setPingAck(ping_ack_builder);
          if (debug_) {
            std::string msg_out = "CONSOLE Sending PING ACK: " + payload.dump();
            ROS_INFO_STREAM_COLOR("cyan", msg_out);
          }
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - consolePingAck [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  consolePingAck [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  consolePingAck [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  consolePingAck [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 


    }
    // visualization Topics
    else if (msg_type == "ego_state") {
      try
      {
            qos = V_QOS;
            EgoState::Builder ego_state_builder = msg.initRoot<EgoState>();
            ego_state_builder.setSignalLight(payload["signalLight"]);
            ego_state_builder.setVehPositionX(payload["vehPositionX"]);
            ego_state_builder.setVehPositionY(payload["vehPositionY"]);
            ego_state_builder.setVehYaw(payload["vehYaw"]);

            ego_state_builder.setTrailerPositionX(payload["trailerPositionX"]);
            ego_state_builder.setTrailerPositionY(payload["trailerPositionY"]);
            ego_state_builder.setTrailerYaw(payload["trailerYaw"]);

            ego_state_builder.setGearStatus(payload["gearStatus"]);
            ego_state_builder.setBrakePercentage(payload["brakePercentage"]);
            ego_state_builder.setVelocity(payload["velocity"]);
            ego_state_builder.setSteeringAngle(payload["steeringAngle"]);
            ego_state_builder.setRemoteEmergencyButtonStatus(payload["remoteEmergencyButtonStatus"]);
            ego_state_builder.setLocalizationStatus(payload["localizationStatus"]);

            ego_state_builder.setHeadLights(payload["headLights"]);
            ego_state_builder.setThrottlePercentage(payload["throttlePercentage"]);
            ego_state_builder.setHornCmdFeedback(payload["hornCmdFeedback"]);
            ego_state_builder.setInSSA(payload["inSSA"]);

            apm_status_.store(payload["apmStatus"]);
            session_job_status_.store(payload["sessionJobStatus"]);
            
            if (debug_) {
              std::string ego_state_msg = payload.dump();
              writeMsgToFile("ego_state.txt", ego_state_msg, "outgoing_payload", true);
            }
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - ego_state [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  ego_state [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  ego_state [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  ego_state [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
    } else if (msg_type == "vehicle_control_mode") {
      try
      {
        qos = V_QOS;
        VehicleControlMode::Builder vcm_builder = msg.initRoot<VehicleControlMode>();

        vcm_builder.setMode(payload["mode"]);
        vcm_builder.setPauseResume(payload["pauseResume"]);
        vehicle_control_mode_.store(payload["mode"]);
        // std::string vcm_msg = payload.dump();
        // ROS_WARN_STREAM("vcm_msg:" << vcm_msg);
        // writeMsgToFile("vehicle_control_mode.txt", vcm_msg, "outgoing_payload", true);
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - vehicle_control_mode [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  vehicle_control_mode [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  vehicle_control_mode [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  vehicle_control_mode [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
    } else if (msg_type == "avcs_job_status") {
      try {
            qos = V_QOS;
            std::string job_status = payload["jobStatus"];
            avcs_job_status_ = std::stoi(job_status);
            
            ::capnp::MallocMessageBuilder my_msg_builder;
            Message::Builder my_message = my_msg_builder.initRoot<Message>();

            kj::ArrayPtr<const kj::byte> payloadSerializedData(
                reinterpret_cast<const kj::byte*>(job_status.data()), job_status.size());
            capnp::Data::Reader payloadData(
                reinterpret_cast<const capnp::byte*>(payloadSerializedData.begin()),
                payloadSerializedData.size());

            uint16_t version = 1;
            uint64_t ts = getTimestamp();
            my_message.setTimestamp(ts);
            my_message.setVersion(version);
            my_message.setType(topic_name);
            my_message.setPayload(payloadData);
            my_message.setUncompressedPayloadSize(payloadSerializedData.size());

            kj::Array<capnp::word> packed_data = ::capnp::messageToFlatArray(my_msg_builder);

            // Convert the packed data to a string for publishing
            std::string serialized_message(reinterpret_cast<const char*>(packed_data.begin()),
                                          packed_data.size() * sizeof(capnp::word));

            std::string pub_msg = serialized_message;
            std::string msg_sender = "vehicle";

            publishMQTTMessage(msg_type, pub_msg, topic_name, qos, msg_sender, false);
            writeMsgToFile("avcs_job_status.txt", job_status, "outgoing_payload", true);
            return;

      } 
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - avcs_job_status [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  avcs_job_status [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  avcs_job_status [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  avcs_job_status [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
    
    } else if (msg_type == "ehmi_info") {
      try 
      {
          qos = V_QOS;
          std::string ehmi_info = "";
          if (payload["ehmiInfo"].is_string())
          {
            ehmi_info = payload["ehmiInfo"];
          }
         
          // if (ehmi_info == "")
          // {
          //   ROS_ERROR("serializeMsgFromVehicle:ehmiInfo is empty");
          //   errorRaised = true; 
          //   return;
          // }
         
          ::capnp::MallocMessageBuilder my_msg_builder;
          Message::Builder my_message = my_msg_builder.initRoot<Message>();

          kj::ArrayPtr<const kj::byte> payloadSerializedData(
              reinterpret_cast<const kj::byte*>(ehmi_info.data()), ehmi_info.size());
          capnp::Data::Reader payloadData(
              reinterpret_cast<const capnp::byte*>(payloadSerializedData.begin()),
              payloadSerializedData.size());

          uint16_t version = 1;
          uint64_t ts = getTimestamp();
          my_message.setTimestamp(ts);
          my_message.setVersion(version);
          my_message.setType(topic_name);
          my_message.setPayload(payloadData);
          my_message.setUncompressedPayloadSize(payloadSerializedData.size());

          kj::Array<capnp::word> packed_data = ::capnp::messageToFlatArray(my_msg_builder);

          // Convert the packed data to a string for publishing
          std::string serialized_message(reinterpret_cast<const char*>(packed_data.begin()),
                                        packed_data.size() * sizeof(capnp::word));

          std::string pub_msg = serialized_message;
          std::string msg_sender = "vehicle";

          publishMQTTMessage(msg_type, pub_msg, topic_name, qos, msg_sender, false);
          ROS_ERROR_STREAM("Writing:"<<ehmi_info);
          writeMsgToFile("ehmi_info.txt", ehmi_info, "outgoing_payload", true);
          return;
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - ehmi_info [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  ehmi_info [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  ehmi_info [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  ehmi_info [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
    } else if (msg_type == "avcs_destination_from_ui") {
      try 
      {
          qos = V_QOS;
          std::string destination = "";
          if (payload["destination"].is_string())
          {
            destination = payload["destination"];
          }

          // if  (destination == "")
          // {
          //   ROS_ERROR("serializeMsgFromVehicle:destination is empty");
          //   errorRaised = true; 
          //   return;
          // }
          ::capnp::MallocMessageBuilder my_msg_builder;
          Message::Builder my_message = my_msg_builder.initRoot<Message>();

          kj::ArrayPtr<const kj::byte> payloadSerializedData(
              reinterpret_cast<const kj::byte*>(destination.data()), destination.size());
          capnp::Data::Reader payloadData(
              reinterpret_cast<const capnp::byte*>(payloadSerializedData.begin()),
              payloadSerializedData.size());

          uint16_t version = 1;
          uint64_t ts = getTimestamp();
          my_message.setTimestamp(ts);
          my_message.setVersion(version);
          my_message.setType(topic_name);
          my_message.setPayload(payloadData);
          my_message.setUncompressedPayloadSize(payloadSerializedData.size());

          kj::Array<capnp::word> packed_data = ::capnp::messageToFlatArray(my_msg_builder);

          // Convert the packed data to a string for publishing
          std::string serialized_message(reinterpret_cast<const char*>(packed_data.begin()),
                                        packed_data.size() * sizeof(capnp::word));

          std::string pub_msg = serialized_message;
          std::string msg_sender = "vehicle";

          publishMQTTMessage(msg_type, pub_msg, topic_name, qos, msg_sender, false);

          writeMsgToFile("avcs_destination_from_ui.txt", destination, "outgoing_payload", true);
          return;
      } 
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - avcs_destination_from_ui [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  avcs_destination_from_ui [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  avcs_destination_from_ui [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  avcs_destination_from_ui [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
    } else if (msg_type == "traffic_light_status") {
      try
      {
          qos = V_QOS;
          TrafficLight::Builder tl_builder = msg.initRoot<TrafficLight>();

          tl_builder.setColor(payload["color"]);
          // std::string tl_msg = payload.dump();
          // writeMsgToFile("traffic_light_status.txt", tl_msg, "outgoing_payload", true);
      }  
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - traffic_light_status [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  traffic_light_status [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  traffic_light_status [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  traffic_light_status [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      
    } else if (msg_type == "tf")
    {
      try
      {
        qos = V_QOS;

        Transform::Builder tf_builder = msg.initRoot<Transform>();

        tf_builder.setStamp(payload["stamp"]);
        
        tf_builder.setTranslationX(payload["translationX"]); 
        tf_builder.setTranslationY(payload["translationY"]); 
        tf_builder.setTranslationZ(payload["translationZ"]); 
        
        tf_builder.setRotationX(payload["rotationX"]);
        tf_builder.setRotationY(payload["rotationY"]);
        tf_builder.setRotationZ(payload["rotationZ"]);
        tf_builder.setRotationW(payload["rotationW"]);
        std::string frame_id = payload["frameId"];
        tf_builder.setFrameId(frame_id);

        std::string child_frame_id = payload["childFrameId"];
        tf_builder.setChildFrameId(child_frame_id);
      }  

      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - tf [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  tf [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  tf [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  tf [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
    } 
    //MARKER
    else if ( msg_type =="most_constrained_object" )
    {
    
     try
      {
        qos = V_QOS;
        Marker::Builder marker_builder = msg.initRoot<Marker>();
        nlohmann::json j = payload["marker"];
   
        uint64_t stamp =  j["stamp"];
        marker_builder.setStamp(stamp);
        
        int32_t id =  j["id"]; 
        marker_builder.setId(id);
          
        uint8_t marker_type = j["type"];
        marker_builder.setType(marker_type);
        
        uint8_t action = j["action"];
        marker_builder.setAction(action);


        float positionX = j["pose"]["position"]["x"];
        float positionY = j["pose"]["position"]["y"];
        float positionZ = j["pose"]["position"]["z"];
        marker_builder.setPositionX(positionX);
        marker_builder.setPositionY(positionY);
        marker_builder.setPositionZ(positionZ);

        float orientationX = j["pose"]["orientation"]["x"];
        float orientationY = j["pose"]["orientation"]["y"];
        float orientationZ = j["pose"]["orientation"]["z"];
        float orientationW = j["pose"]["orientation"]["w"];
        marker_builder.setOrientationX(orientationX);
        marker_builder.setOrientationY(orientationY);
        marker_builder.setOrientationZ(orientationZ);
        marker_builder.setOrientationW(orientationW);

        float scaleX = j["scale"]["x"];
        float scaleY = j["scale"]["y"];
        float scaleZ = j["scale"]["z"];
        marker_builder.setScaleX(scaleX);
        marker_builder.setScaleY(scaleY);
        marker_builder.setScaleZ(scaleZ);

        float colorR = j["color"]["r"];
        float colorG = j["color"]["g"];
        float colorB = j["color"]["b"];
        float colorA = j["color"]["a"];
        marker_builder.setColorR(colorR);
        marker_builder.setColorG(colorG);
        marker_builder.setColorB(colorB);
        marker_builder.setColorA(colorA);
          
        std::string text = j["text"];
        marker_builder.setText(text);
          
        std::string ns = j["ns"];
        marker_builder.setNs(ns);

        auto point_size = j["points"].size();  
        ::capnp::List<float>::Builder point_list = marker_builder.initPoints(point_size);
        
        for (size_t k = 0; k < point_size; k++) {
           point_list.set(k,   j["points"][k]);
        }

  
      }  
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - "<<msg_type <<" [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR - "<<msg_type <<" [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR - "<<msg_type <<"  [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR - "<<msg_type <<"  [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
  
    }
      //MARKERS
    else if (msg_type =="objects" || 
             msg_type =="crosswalk_vis" ||
             msg_type =="traffic_jam_lanes_vis" ||
             msg_type =="road_global_vis" || msg_type =="objects_of_interest" ||
             msg_type =="hatch_cover_detection_box" || msg_type =="rtg_detection_box")
    {
    
      try
      {
        qos = V_QOS;
        MarkerArray::Builder marker_array_builder = msg.initRoot<MarkerArray>();
        auto markers = payload["markers"];
        int markers_size = markers.size();
        ::capnp::List<Marker>::Builder marker_list = marker_array_builder.initMarkers(markers_size);

      
        for (size_t i = 0; i < markers.size(); i++) 
        {

          nlohmann::json j = markers[i];
          Marker::Builder marker_builder = marker_list[i];
          uint64_t stamp =  j["stamp"];
          marker_builder.setStamp(stamp);
        
          int32_t id =  j["id"]; 
          marker_builder.setId(id);
          
          uint8_t marker_type = j["type"];
          marker_builder.setType(marker_type);
        
          uint8_t action = j["action"];
          marker_builder.setAction(action);


          float positionX = j["pose"]["position"]["x"];
          float positionY = j["pose"]["position"]["y"];
          float positionZ = j["pose"]["position"]["z"];
          marker_builder.setPositionX(positionX);
          marker_builder.setPositionY(positionY);
          marker_builder.setPositionZ(positionZ);

          float orientationX = j["pose"]["orientation"]["x"];
          float orientationY = j["pose"]["orientation"]["y"];
          float orientationZ = j["pose"]["orientation"]["z"];
          float orientationW = j["pose"]["orientation"]["w"];
          marker_builder.setOrientationX(orientationX);
          marker_builder.setOrientationY(orientationY);
          marker_builder.setOrientationZ(orientationZ);
          marker_builder.setOrientationW(orientationW);

          float scaleX = j["scale"]["x"];
          float scaleY = j["scale"]["y"];
          float scaleZ = j["scale"]["z"];
          marker_builder.setScaleX(scaleX);
          marker_builder.setScaleY(scaleY);
          marker_builder.setScaleZ(scaleZ);

          float colorR = j["color"]["r"];
          float colorG = j["color"]["g"];
          float colorB = j["color"]["b"];
          float colorA = j["color"]["a"];
          marker_builder.setColorR(colorR);
          marker_builder.setColorG(colorG);
          marker_builder.setColorB(colorB);
          marker_builder.setColorA(colorA);
          
          std::string text = j["text"];
          marker_builder.setText(text);
          
          std::string ns = j["ns"];
          marker_builder.setNs(ns);

          auto point_size = j["points"].size();  
          ::capnp::List<float>::Builder point_list = marker_builder.initPoints(point_size);
          for (size_t k = 0; k < point_size; k++) {
            point_list.set(k,   j["points"][k]);
          }

        }
    
      }  
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - "<<msg_type <<" [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR - "<<msg_type <<" [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR - "<<msg_type <<"  [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR - "<<msg_type <<"  [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
  
    } else if (msg_type == "route_plan") {
     try
     { 
      qos = V_QOS;
      Path::Builder path_builder = msg.initRoot<Path>();

      path_builder.setTimestamp(payload["timestamp"]);
      auto vec_segment_id   = payload["segmentId"]; 
      auto vec_points_x     = payload["discretizedPathPointsX"]; 
      auto vec_points_y     = payload["discretizedPathPointsY"]; 
      auto vec_points_yaw   = payload["discretizedPathPointsYaw"]; 

    // Initialize the list with the desired size
      const size_t num_segment_id = vec_segment_id.size(); 
      const size_t num_points = vec_points_x.size(); 
      
      ::capnp::List<uint32_t>::Builder segment_id_list = path_builder.initSegmentId(num_segment_id);
      ::capnp::List<float>::Builder x_list   = path_builder.initDiscretizedPathPointsX(num_points);
      ::capnp::List<float>::Builder y_list   = path_builder.initDiscretizedPathPointsY(num_points);
      ::capnp::List<float>::Builder yaw_list = path_builder.initDiscretizedPathPointsYaw(num_points);
      
      for (size_t i = 0; i < num_segment_id; i++)
      {
        segment_id_list.set(i, vec_segment_id[i]);
      } 

      for (size_t i = 0; i < num_points; i++)
      {
        x_list.set(i,   vec_points_x[i]);
        y_list.set(i,   vec_points_y[i]); 
        yaw_list.set(i, vec_points_yaw[i]);
      } 

     
      std::string path_msg = payload.dump();
      writeMsgToFile("route_plan.txt", path_msg, "outgoing_payload", true);
     }  
     catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - route_plan [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
     }
     catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  route_plan [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
     }     
     catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  route_plan [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
     } 
     catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  route_plan [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
     }
    }
    
    else if (msg_type == "predicted_footprint") {
     try
     { 
      qos = V_QOS;
      Polygon::Builder polygon_builder = msg.initRoot<Polygon>();

      polygon_builder.setStamp(payload["stamp"]);
      polygon_builder.setIncludesZ(payload["includesZ"]);

      auto vec_points = payload["payload"]; 

    // Initialize the list with the desired size
      const size_t numElements = vec_points.size(); 
      ::capnp::List<float>::Builder payloadList = polygon_builder.initPayload(numElements);
      for (size_t i = 0; i < vec_points.size(); i++)
      {
        payloadList.set(i, vec_points[i]);
      } 
  
      std::string polygon_msg = payload.dump();
      writeMsgToFile("predicted_footprint.txt", polygon_msg, "outgoing_payload", true);
     }  
     catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - predicted_footprint [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
     }
     catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  predicted_footprint [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
     }     
     catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  predicted_footprint [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
     } 
     catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  predicted_footprint [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
     }
    } else if (msg_type == "vertical_points") {
      try
      {
        
        qos = V_QOS;
        PCD::Builder pcd_builder = msg.initRoot<PCD>();

        pcd_builder.setStamp(payload["stamp"]);
        pcd_builder.setIncludesZ(payload["includesZ"]);
        pcd_builder.setIncludesIntensity(payload["includesIntensity"]);
        auto vec_points = payload["payload"]; 

      // Initialize the list with the desired size
        const size_t numElements = vec_points.size(); 
        ::capnp::List<float>::Builder payloadList = pcd_builder.initPayload(numElements);
        for (size_t i = 0; i < vec_points.size(); i++)
        {
          payloadList.set(i, vec_points[i]);
        } 
      }  
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - vertical_points [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  vertical_points [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  vertical_points [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  vertical_points [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
    } else if (msg_type == "on_road_pc") {
      try
      {
        qos = V_QOS;
        PCD::Builder pcd_builder = msg.initRoot<PCD>();

        pcd_builder.setStamp(payload["stamp"]);
        pcd_builder.setIncludesZ(payload["includesZ"]);
        pcd_builder.setIncludesIntensity(payload["includesIntensity"]);
        auto vec_points = payload["payload"]; 
        const size_t numElements = vec_points.size(); 
        ::capnp::List<float>::Builder payloadList = pcd_builder.initPayload(numElements);
        for (size_t i = 0; i < vec_points.size(); i++)
        {
          payloadList.set(i, vec_points[i]);
        } 
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - on_road_pc [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  on_road_pc [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  on_road_pc [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  on_road_pc [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
        
     
    } else if (msg_type == "curb_pc") {
      try
      {
        qos = V_QOS;
        PCD::Builder pcd_builder = msg.initRoot<PCD>();

        pcd_builder.setStamp(payload["stamp"]);
        pcd_builder.setIncludesZ(payload["includesZ"]);
        pcd_builder.setIncludesIntensity(payload["includesIntensity"]);
        auto vec_points = payload["payload"]; 

        const size_t numElements = vec_points.size(); 
        ::capnp::List<float>::Builder payloadList = pcd_builder.initPayload(numElements);
        for (size_t i = 0; i < vec_points.size(); i++)
        {
          payloadList.set(i, vec_points[i]);
        } 
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - curb_pc [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  curb_pc [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  curb_pc [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  curb_pc [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
    } else if (msg_type == "most_constrained_points") {
      try
      {
        qos = V_QOS;
        PCD::Builder pcd_builder = msg.initRoot<PCD>();

        pcd_builder.setStamp(payload["stamp"]);
        pcd_builder.setIncludesZ(payload["includesZ"]);
        pcd_builder.setIncludesIntensity(payload["includesIntensity"]);
        auto vec_points = payload["payload"]; 

    //  // Initialize the list with the desired size
        const size_t numElements = vec_points.size(); 
        ::capnp::List<float>::Builder payloadList = pcd_builder.initPayload(numElements);
        for (size_t i = 0; i < vec_points.size(); i++)
        {
          payloadList.set(i, vec_points[i]);
        } 
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - most_constrained_points [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  most_constrained_points [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  most_constrained_points [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  most_constrained_points [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
    } else if (msg_type == "road_intensity_detection") {
      try
      {
        qos = V_QOS;
        PCD::Builder pcd_builder = msg.initRoot<PCD>();

        pcd_builder.setStamp(payload["stamp"]);
        pcd_builder.setIncludesZ(payload["includesZ"]);
        pcd_builder.setIncludesIntensity(payload["includesIntensity"]);
        auto vec_points = payload["payload"]; 

        const size_t numElements = vec_points.size(); 
        ::capnp::List<float>::Builder payloadList = pcd_builder.initPayload(numElements);
        for (size_t i = 0; i < vec_points.size(); i++)
        {
          payloadList.set(i, vec_points[i]);
        } 
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - road_intensity_detection [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
      }
      catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  road_intensity_detection [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  road_intensity_detection [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  road_intensity_detection [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
    
    }  
    else if (msg_type == "ops_response") 
    {
      try
      {
        qos = V_QOS;

        OpsResponse::Builder res_builder = msg.initRoot<OpsResponse>();

        res_builder.setReqReceivedTime(payload["reqReceivedTime"]);//UInt64;
        res_builder.setOperationStatus(payload["operationStatus"]); //Bool
        res_builder.setOperationType(payload["operationType"]); //UInt8
        std::string reason = payload["reason"];
        res_builder.setReason(reason); //Text
        std::string res_msg = payload.dump();
        writeMsgToFile("ops_response.txt", res_msg, "outgoing_payload", true);
      }  
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - ops_response [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
     }
     catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  ops_response [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  ops_response [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  ops_response [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
    } 
    
    /****OPS_REQUEST***/
    else if (msg_type == "remote_ops_request") {
      try
      {
        qos = V_QOS;
        RemoteOpsRequest::Builder req_builder = msg.initRoot<RemoteOpsRequest>();
        
        
        /**RED ALERTS**/
        std::vector<uint32_t> vec_red_alerts = payload["redAlerts"].get<std::vector<uint32_t> >();
        const size_t red_alerts_num = vec_red_alerts.size(); 
        auto readAlertsList = req_builder.initRedAlerts(red_alerts_num);
        for (size_t i = 0; i < red_alerts_num; i++)
        {
          readAlertsList.set(i, vec_red_alerts[i]);
        } 
        /**RED ACTIONABLE ALERTS**/
        std::vector<uint32_t> vec_red_act_alerts = payload["redActionableAlerts"].get<std::vector<uint32_t> >();
        const size_t red_act_alerts_num = vec_red_act_alerts.size(); 
        auto redActAlertsList = req_builder.initRedActionableAlerts(red_act_alerts_num);
        for (size_t i = 0; i < red_act_alerts_num; i++)
        {
          redActAlertsList.set(i, vec_red_act_alerts[i]);
        } 
        /*AMBER ALERTS*/
        std::vector<uint32_t> vec_amber_alerts = payload["amberAlerts"].get<std::vector<uint32_t> >();
        const size_t amber_alerts_num = vec_amber_alerts.size(); 
        auto amberAlertsList = req_builder.initAmberAlerts(amber_alerts_num);
        for (size_t i = 0; i < amber_alerts_num; i++)
        {
          amberAlertsList.set(i, vec_amber_alerts[i]);
        } 

        /*AMBER ACTIONABLE ALERTS*/
        std::vector<uint32_t> vec_amber_act_alerts = payload["amberActionableAlerts"].get<std::vector<uint32_t> >();
        const size_t amber_act_alerts_num = vec_amber_act_alerts.size(); 
        auto amberActAlertsList = req_builder.initAmberActionableAlerts(amber_act_alerts_num);
        for (size_t i = 0; i < amber_act_alerts_num; i++)
        {
          amberActAlertsList.set(i, vec_amber_act_alerts[i]);
        }

        /*Recommended Actions DEPRECTAED - VALUES will be IGNORED*/
        std::vector<uint8_t> vec_rec_act = payload["recommendedActions"].get<std::vector<uint8_t>>();
        const size_t rec_act_num = vec_rec_act.size(); 
        auto recommendedActionsList = req_builder.initRecommendedActions(rec_act_num);
        for (size_t i = 0; i < rec_act_num; i++)
        {
          recommendedActionsList.set(i, vec_rec_act[i]);
        }
      /*Restricted Actions*/
        std::vector<uint8_t> vec_res_act = payload["restrictedActions"].get<std::vector<uint8_t>>();
        const size_t res_act_num = vec_res_act.size(); 
        auto restrictedActionsList = req_builder.initRestrictedActions(res_act_num);
        for (size_t i = 0; i < res_act_num; i++)
        {
          restrictedActionsList.set(i, vec_res_act[i]);
        }

        /*HighRiskType*/
        std::vector<uint8_t> vec_high_risk_type = payload["highRiskType"].get<std::vector<uint8_t>>();
        const size_t high_risk_type_num = vec_high_risk_type.size(); 
        auto highRiskTypeList = req_builder.initHighRiskType(high_risk_type_num);
        for (size_t i = 0; i < high_risk_type_num; i++)
        {
          highRiskTypeList.set(i, vec_high_risk_type[i]);
        }

        /*HighRiskDist*/
        std::vector<_Float32> vec_high_risk_dist = payload["highRiskDist"].get<std::vector<_Float32>>();
        const size_t high_risk_dist_num = vec_high_risk_dist.size(); 
        auto highRiskDistList = req_builder.initHighRiskDist(high_risk_dist_num);
        for (size_t i = 0; i < high_risk_dist_num; i++)
        {
          highRiskDistList.set(i, vec_high_risk_dist[i]);
        }

        /*MetricTypes*/
        std::vector<uint32_t> vec_metric_types = payload["metricTypes"].get<std::vector<uint32_t>>();
        const size_t metric_types_num = vec_metric_types.size(); 
        auto metricTypesList = req_builder.initMetricTypes(metric_types_num);
        for (size_t i = 0; i < metric_types_num; i++)
        {
          metricTypesList.set(i, vec_metric_types[i]);
        }

        /*MetricValues*/
        std::vector<std::string> vec_metric_val = payload["metricValues"].get<std::vector<std::string>>();
        const size_t metric_val_num = vec_metric_val.size(); 
        auto metricValuesList = req_builder.initMetricValues(metric_val_num);
        for (size_t i = 0; i < metric_val_num; i++)
        {
          metricValuesList.set(i, vec_metric_val[i]);
        }
        
        /*Job done allowed*/
        bool job_done_allowed = payload["jobDoneAllowed"];
        req_builder.setJobDoneAllowed(job_done_allowed);

        std::string req_msg = payload.dump();
        writeMsgToFile("remote_ops_request.txt", req_msg, "outgoing_payload", true);
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - remote_ops_request [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
     }
     catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  remote_ops_request [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  remote_ops_request [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  remote_ops_request [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }
    }
    
    else if (msg_type == "remote_ops_init") {
      try
      {
        qos = V_QOS;
        RemoteOpsInit::Builder ops_init_builder = msg.initRoot<RemoteOpsInit>();
      
        /**metricsType **/
        std::vector<uint8_t> vec_metrics_type = payload["metricsType"].get<std::vector<uint8_t>>();
        int metrics_type_num = vec_metrics_type.size(); 
        capnp::List<uint8_t>::Builder metricsTypeList = ops_init_builder.initMetricsType(metrics_type_num);
        
        for (int i = 0; i < metrics_type_num; i++)
        {
          metricsTypeList.set(i, vec_metrics_type[i]);
        } 
        //ROS_INFO_STREAM("1-metrics_type_num: "<<metrics_type_num);
    
        /**metricsValueType **/
        std::vector<uint8_t> vec_metrics_val_type = payload["metricsValueType"].get<std::vector<uint8_t>>();
        int metrics_val_type_num = vec_metrics_val_type.size(); 
        capnp::List<uint8_t>::Builder metricsValueTypeList = ops_init_builder.initMetricsValueType(metrics_val_type_num);
        for (int i = 0; i < metrics_val_type_num; i++)
        {
          metricsValueTypeList.set(i, vec_metrics_val_type[i]);
        } 
        //ROS_INFO_STREAM("2-vec_metrics_val_type: "<<metrics_val_type_num);        
      
        /** alertType **/
        std::vector<uint32_t> vec_alert_type = payload["alertType"].get<std::vector<uint32_t>>();
        int alert_type_num = vec_alert_type.size(); 
        capnp::List<uint32_t>::Builder alertTypeList = ops_init_builder.initAlertType(alert_type_num);
        for (int i = 0; i < alert_type_num; i++)
        {
          alertTypeList.set(i, vec_alert_type[i]);
        } 
        //ROS_INFO_STREAM("3-vec_alert_type: "<<alert_type_num);        
        /** alertThreshold **/
        std::vector<std::string> vec_alert_thresh = payload["alertThreshold"].get<std::vector<std::string>>();
        int alert_thresh_num = vec_alert_thresh.size(); 
        capnp::List<capnp::Text>::Builder alertThresholdList = ops_init_builder.initAlertThreshold(alert_thresh_num);

        for (int i = 0; i < alert_thresh_num; i++)
        {
          alertThresholdList.set(i, vec_alert_thresh[i]);
        }
        //ROS_INFO_STREAM("4-alert_thresh_num: "<<alert_thresh_num); 
        // for (auto el:vec_alert_thresh)
        // {
        //   ROS_INFO_STREAM("alert_type:"<<el);
        // }
      
      /**alertIds**/
        std::vector<uint32_t> vec_alert_ids = payload["alertIds"].get<std::vector<uint32_t>>();
        int alert_ids_num = vec_alert_ids.size(); 
        capnp::List<uint32_t>::Builder alertIdsList = ops_init_builder.initAlertIds(alert_ids_num);
        for (int i = 0; i < alert_ids_num; i++)
        {
          alertIdsList.set(i, vec_alert_ids[i]);
        } 
        // ROS_INFO_STREAM("5-alert_ids_num: "<<alert_ids_num); 
        // for (auto el:vec_alert_ids)
        // {
        //   ROS_INFO_STREAM("alert_id:"<<static_cast<int>(el));
        // }

        /***alertDescriptions**/
        std::vector<std::string> vec_alert_desc = payload["alertDescriptions"].get<std::vector<std::string>>();
        int alert_desc_num = vec_alert_desc.size(); 
        capnp::List<capnp::Text>::Builder alertDescriptionsList = ops_init_builder.initAlertDescriptions(alert_desc_num);
        for (int i = 0; i < alert_desc_num; i++)
        {
         alertDescriptionsList.set(i, vec_alert_desc[i]);
        }
        // ROS_INFO_STREAM("6-alert_desc_num: "<<alert_desc_num); 
        // for (auto el:vec_alert_desc)
        // {
        //   ROS_INFO_STREAM("alert_desc:"<<el);
        // }

        std::string init_msg = payload.dump();
        writeMsgToFile("remote_ops_init.txt", init_msg, "outgoing_payload", true);
      
      }
      catch (const kj::Exception& e) {
        std::cerr<<"\n1-ERROR - remote_ops_init [serializeMsgFromVehicle][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
        errorRaised = true; 
        return;
     }
     catch (const std::exception& e) {
        std::cerr<<"\n2-ERROR -  remote_ops_init [serializeMsgFromVehicle][std::exception]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }     
      catch (const nlohmann::json::type_error& e) {
        std::cerr<<"\n3-ERROR -  remote_ops_init [serializeMsgFromVehicle][type_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      } 
      catch (const std::logic_error& e) {
        std::cerr<<"\n4-ERROR -  remote_ops_init [serializeMsgFromVehicle][logic_error]:" << e.what()<<std::endl;
        errorRaised = true; 
        return;
      }

    }  
    uint64_t msg_timestamp;
    auto serialized_message = encodeMessage(topic_name, msg, msg_timestamp, errorRaised);

    if (errorRaised)
    {
        return;
    }

    if (qos == C_QOS) {
      auto crc32 = CRC32(serialized_message);
      serialized_message.append(reinterpret_cast<const char*>(&crc32), sizeof(crc32));
    }
    std::string pub_msg = serialized_message;
    std::string msg_sender = "vehicle";
    
    
    publishMQTTMessage(msg_type, pub_msg, topic_name, qos, msg_sender, isVehiclePingAck);

    if (isVehiclePingReq) {
      if (debug_) {
        nlohmann::json write_ping_msg_json;
        write_ping_msg_json["vehicle_sent_req_ts"] = msg_timestamp;
        write_ping_msg_json["seq"] = payload["seq"];
        std::string write_ping_msg = write_ping_msg_json.dump() + "\n";
        writeMsgToFile("ping.txt", write_ping_msg, "incoming_payload", true);
      }
    }
  }

  catch (nlohmann::json::type_error& e) {
    std::cerr << "1-serializeMsgFromVehicle: Exception[type_error]-msgType: " <<msg_type<<'-'<< e.what() << std::endl;
     errorRaised = true; 
  }
  catch (const std::exception& e) {
    std::cerr << "2-serializeMsgFromVehicle: Exception[std]-msgType: " <<msg_type<<'-'<< e.what() << std::endl;
     errorRaised = true; 
  }      
  catch (const kj::Exception& e) {
    std::cerr<<"3-serializeMsgFromVehicle: Exception[kj::Exception]-msgType: " << msg_type<< '-'<< e.getDescription().cStr()<<std::endl;
    errorRaised = true; 
  }
  catch (const std::logic_error& e) {
    std::cerr<<"4-serializeMsgFromVehicle: Exception[logic_error]-msgType: " << msg_type<< '-'<<e.what() << std::endl;
    errorRaised = true; 
  }    

};

bool RemoteOps::deserializeMsgFromConsole(mqtt::const_message_ptr msg,
                                          nlohmann::json& json_deserialized_msg,
                                          bool& errorRaised,
                                          bool& rejected,
                                          std::string& reject_reason) {
 
   
  std::string serializedMsg = msg->get_payload_str();

  std::string topic_name = msg->get_topic();
 
  try {
    kj::ArrayPtr<const capnp::word> capnpArray(
        reinterpret_cast<const capnp::word*>(serializedMsg.data()),
        serializedMsg.size() / sizeof(capnp::word));

    capnp::FlatArrayMessageReader array_message_reader(capnpArray);
    // Access the root of message
    Message::Reader message_reader = array_message_reader.getRoot<Message>();
    json_deserialized_msg["topicName"] = topic_name;
    json_deserialized_msg["timestamp"] = message_reader.getTimestamp();
    json_deserialized_msg["type"] = message_reader.getType().cStr();
    json_deserialized_msg["version"] = message_reader.getVersion();
    json_deserialized_msg["payloadSize"] = message_reader.getUncompressedPayloadSize();
    // Access the root of payload
    auto payload_reader = message_reader.getPayload();

    // Obtain the pointer to the underlying data and its size
    const auto* payload_ptr = payload_reader.asBytes().begin();
    const size_t payload_size = payload_reader.asBytes().size() / sizeof(capnp::word);
    // Create a kj::ArrayPtr<const capnp::word> from the underlying data
    kj::ArrayPtr<const capnp::word> payload_array_ptr(
        reinterpret_cast<const capnp::word*>(payload_ptr), payload_size);
    std::string type;
    capnp::FlatArrayMessageReader payload_msg_reader(payload_array_ptr);

    std::string msg_type = json_deserialized_msg["type"];
    std::size_t found = topic_name.rfind('/');
    if (found != std::string::npos) {
      msg_type = topic_name.substr(found + 1);
    }

    if (msg_type == "linkup") {
      LinkUp::Reader linkup_reader = payload_msg_reader.getRoot<LinkUp>();

      if (linkup_reader.getCommand().hasLinkUpFromConsoleAck()) {
        ROS_WARN_STREAM("*****VEHICLE LINKUP ACK MESSAGE*******");
        return false;
      } else {
        ROS_WARN_STREAM("*****CONSOLE LINKUP  REQUEST*******");
      }

      view_type_ = linkup_reader.getCommand().getLinkUpFromConsole().getViewType();
      console_mode_ = linkup_reader.getCommand().getLinkUpFromConsole().getConsoleMode();
      console_id_ = linkup_reader.getCommand().getLinkUpFromConsole().getConsoleId().cStr();
      vehicle_id_ = linkup_reader.getCommand().getLinkUpFromConsole().getVehicleId().cStr();
      json_deserialized_msg["payload"]["consoleID"] = console_id_;
      json_deserialized_msg["payload"]["vehicleID"] = vehicle_id_;
      json_deserialized_msg["payload"]["viewType"] = view_type_;
      json_deserialized_msg["payload"]["consoleMode"] = console_mode_;

    } else if (msg_type == "breaklink") {
      BreakLink::Reader breaklink_reader = payload_msg_reader.getRoot<BreakLink>();
      if (breaklink_reader.getCommand().hasBreakLinkFromConsoleAck()) {
        console_id_ =
            breaklink_reader.getCommand().getBreakLinkFromConsoleAck().getConsoleId().cStr();
        vehicle_id_ =
            breaklink_reader.getCommand().getBreakLinkFromConsoleAck().getVehicleId().cStr();
        json_deserialized_msg["payload"]["consoleID"] = console_id_;
        json_deserialized_msg["payload"]["vehicleID"] = vehicle_id_;
        ROS_WARN_STREAM("***** BREAKLINK ACK MESSAGE-DO NOTHING*******");
        return false;
      } else  // Breaklink Request
      {
        if (isVehicleBreakLinkRequest_) {
          console_id_ =
              breaklink_reader.getCommand().getBreakLinkFromConsole().getConsoleId().cStr();
          vehicle_id_ =
              breaklink_reader.getCommand().getBreakLinkFromConsole().getVehicleId().cStr();
          BreakLinkReason breaklink_reason =
              breaklink_reader.getCommand().getBreakLinkFromConsole().getBreakLinkReason();

          json_deserialized_msg["payload"]["consoleID"] = console_id_;
          json_deserialized_msg["payload"]["vehicleID"] = vehicle_id_;
          json_deserialized_msg["payload"]["breakLinkReason"] = breaklink_reason;
          ROS_WARN_STREAM("*****VEHICLE BREAKLINK REQUEST-DO NOTHING*******");
          return false;
        }

        // Console Breaklink Request
        console_id_ = breaklink_reader.getCommand().getBreakLinkFromConsole().getConsoleId().cStr();
        vehicle_id_ = breaklink_reader.getCommand().getBreakLinkFromConsole().getVehicleId().cStr();
        breaklink_reason_ =
            breaklink_reader.getCommand().getBreakLinkFromConsole().getBreakLinkReason();

        json_deserialized_msg["payload"]["consoleID"] = console_id_;
        json_deserialized_msg["payload"]["vehicleID"] = vehicle_id_;
        json_deserialized_msg["payload"]["breakLinkReason"] = breaklink_reason_;
        ROS_WARN_STREAM("*****CONSOLE BREAKLINK REQUEST*******");
      }

    } else if (msg_type == "teardown") {
      Teardown::Reader teardown_reader = payload_msg_reader.getRoot<Teardown>();

      if (teardown_reader.getCommand().hasTeardownFromConsoleAck()) {
        ROS_WARN_STREAM("*****VEHICLE TEARDOWN ACK MESSAGE*******");
        return false;
      } else {
        ROS_WARN_STREAM("*****CONSOLE TEARDOWN REQUEST*******");
      }

      console_id_ = teardown_reader.getCommand().getTeardownFromConsole().getConsoleId().cStr();
      vehicle_id_ = teardown_reader.getCommand().getTeardownFromConsole().getVehicleId().cStr();
      json_deserialized_msg["payload"]["consoleID"] = console_id_;
      json_deserialized_msg["payload"]["vehicleID"] = vehicle_id_;
    } else if (msg_type == "ping") {
      PingMessage::Reader ping_message_reader = payload_msg_reader.getRoot<PingMessage>();
      auto sender = ping_message_reader.getSender().cStr();
      auto receiver = ping_message_reader.getReceiver().cStr();
      auto seq = ping_message_reader.getSeq();
      json_deserialized_msg["payload"]["sender"] = sender;
      json_deserialized_msg["payload"]["receiver"] = receiver;
      json_deserialized_msg["payload"]["seq"] = seq;

      if (ping_message_reader.getType().hasPingAck() && receiver == console_id_) {
        // vehicle -> console
        //  if (debug_)
        //  {
        //    ROS_WARN_STREAM("[VEHICLE->CONSOLE] PING ACK message");
        //  }

        return false;
      } else if (ping_message_reader.getType().hasPingAck() && receiver == vehicle_id_) {
        // if (debug_)
        // {
        //   ROS_WARN_STREAM("[CONSOLE->VEHICLE] PING ACK message");
        // }
        json_deserialized_msg["payload"]["requestLatency"] =
            ping_message_reader.getType().getPingAck().getRequestLatency();
        return false;
      } else if (receiver == vehicle_id_) {
        if (debug_) {
          ROS_WARN_STREAM("[CONSOLE -> VEHICLE] PING REQUEST");
        }
      } else if (receiver == console_id_) {
        //  if (debug_)
        //  {
        //    ROS_WARN_STREAM("[VEHICLE->CONSOLE] PING REQUEST");
        //  }
        return false;
      }
    } else if (msg_type == "emergency_brake_command") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }
      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      bool apply_brake = ops_reader.getCommand().getEmergencyBrakeCmd().getApplyBrake();
      // std::cout << "\tPayload:" << std::endl;
      std::string applyBrake_str = (apply_brake ? "True" : "False");
      // std::cout << "\t  Emergency brake cmd: " << applyBrake_str << std::endl;
      json_deserialized_msg["payload"]["applyBrake"] = apply_brake;
    } else if (msg_type == "cmd_signal_light") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      int state = ops_reader.getCommand().getHazardLightCmd().getState();
      json_deserialized_msg["payload"]["state"] = state;
    } else if (msg_type == "cmd_head_light_remote") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      int state = ops_reader.getCommand().getHeadLightCmd().getState();
      // std::cout << "\tPayload:" << std::endl;
      // std::cout << "\t  State: " << state << std::endl;
      json_deserialized_msg["payload"]["state"] = state;
    } else if (msg_type == "cmd_horn") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      bool activateHorn = ops_reader.getCommand().getHornCmd().getActivateHorn();
      // std::cout << "\tPayload:" << std::endl;
      // std::cout << "\t  Activate Horn: " << std::boolalpha << activateHorn << std::endl;
      json_deserialized_msg["payload"]["activateHorn"] = activateHorn;
    } else if (msg_type == "traffic_light_override") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      int type = ops_reader.getCommand().getTrafficLightOverrideCmd().getType();
      json_deserialized_msg["payload"]["type"] = type;

    } else if (msg_type == "stop") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      bool pause = ops_reader.getCommand().getPauseResumeCmd().getPause();
      // std::cout << "\tPayload:" << std::endl;
      // std::cout << "\t  Pause: " << std::boolalpha << pause << std::endl;
      json_deserialized_msg["payload"]["pause"] = pause;
    } else if (msg_type == "precedence_override") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      bool isGo = ops_reader.getCommand().getPrecedenceOverrideCmd().getGo();
      // std::cout << "\tPayload:" << std::endl;
      std::string go_str = (isGo ? "Go" : "Stop");
      // std::cout << "\t Precedence Override cmd: " << go_str << std::endl;
      json_deserialized_msg["payload"]["go"] = isGo;
    } else if (msg_type == "goal") {  // set destination cmd

      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      std::string block_name = ops_reader.getCommand().getSetDestinationCmd().getBlockName();
      uint8_t block_id = ops_reader.getCommand().getSetDestinationCmd().getBlockId();
      uint8_t lane_id = ops_reader.getCommand().getSetDestinationCmd().getLaneId();
      uint8_t slot_id = ops_reader.getCommand().getSetDestinationCmd().getSlotId();
      uint8_t container_type = ops_reader.getCommand().getSetDestinationCmd().getContainerType();
      uint8_t job_type = ops_reader.getCommand().getSetDestinationCmd().getJobType();

      json_deserialized_msg["payload"]["blockName"] = block_name;
      json_deserialized_msg["payload"]["blockId"] = block_id;
      json_deserialized_msg["payload"]["laneId"] = lane_id;
      json_deserialized_msg["payload"]["slotId"] = slot_id;
      json_deserialized_msg["payload"]["containerType"] = container_type;
      json_deserialized_msg["payload"]["jobType"] = job_type;
    } else if (msg_type == "adjust_position") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      auto w = ops_reader.getCommand().getAdjustPositionByDistanceCmd().getCommandType().which();
      if (w == 0)  // distance
      {
        float distance =
            ops_reader.getCommand().getAdjustPositionByDistanceCmd().getCommandType().getDistance();
        json_deserialized_msg["payload"]["distance"] = distance;
      } else {
        bool safety_override = ops_reader.getCommand()
                                   .getAdjustPositionByDistanceCmd()
                                   .getCommandType()
                                   .getSafetyOverride();
        json_deserialized_msg["payload"]["safetyOverride"] = safety_override;
      }
    } else if (msg_type == "non_yard_goal") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      uint32_t dest = ops_reader.getCommand().getSetNonYardDestinationCmd().getDestination();
      json_deserialized_msg["payload"]["destination"] = dest;
    } else if (msg_type == "location") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      auto x = ops_reader.getCommand().getRelocalizationCmd().getX();
      auto y = ops_reader.getCommand().getRelocalizationCmd().getY();
      auto yaw = ops_reader.getCommand().getRelocalizationCmd().getYaw();

      json_deserialized_msg["payload"]["x"] = x;
      json_deserialized_msg["payload"]["y"] = y;
      json_deserialized_msg["payload"]["yaw"] = yaw;
    } else if (msg_type == "manual_push_remote") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      auto engage = ops_reader.getCommand().getNudgeForwardCmd().getEngage();
      auto nudge_instance_id = ops_reader.getCommand().getNudgeForwardCmd().getNudgeInstanceId();

      json_deserialized_msg["payload"]["engage"] = engage;
      json_deserialized_msg["payload"]["nudgeInstanceId"] = nudge_instance_id;
    } else if (msg_type == "override_path") {
      if (!linkup_status_) {
        std::string msg_out =
            "[" + msg_type + "] NOT A VALID CONSOLE REQUEST. LINKUP STATUS: FALSE";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
        return false;
      }

      RemoteOpsCommand::Reader ops_reader = payload_msg_reader.getRoot<RemoteOpsCommand>();
      uint64_t timestamp =
          ops_reader.getCommand().getTrajectoryOverrideCmd().getPath().getTimestamp();

      ::capnp::List<uint32_t>::Reader segmentReader =
          ops_reader.getCommand().getTrajectoryOverrideCmd().getPath().getSegmentId();
      ::capnp::List<float>::Reader xReader =
          ops_reader.getCommand().getTrajectoryOverrideCmd().getPath().getDiscretizedPathPointsX();
      ::capnp::List<float>::Reader yReader =
          ops_reader.getCommand().getTrajectoryOverrideCmd().getPath().getDiscretizedPathPointsY();
      ::capnp::List<float>::Reader yawReader = ops_reader.getCommand()
                                                   .getTrajectoryOverrideCmd()
                                                   .getPath()
                                                   .getDiscretizedPathPointsYaw();

      json_deserialized_msg["payload"]["timestamp"] = timestamp;

      for (int i = 0; i < segmentReader.size(); ++i) {
        ROS_WARN_STREAM("segment " << i << ": " << segmentReader[i]);
        json_deserialized_msg["payload"]["segmentId"].push_back(segmentReader[i]);
      }

      for (int i = 0; i < xReader.size(); ++i) {
        ROS_WARN_STREAM("x: " << i << ": " << xReader[i]);
        json_deserialized_msg["payload"]["discretizedPathPointsX"].push_back(xReader[i]);
      }

      for (int i = 0; i < yReader.size(); ++i) {
        ROS_WARN_STREAM("y: " << i << ": " << yReader[i]);
        json_deserialized_msg["payload"]["discretizedPathPointsY"].push_back(yReader[i]);
      }

      for (int i = 0; i < yawReader.size(); ++i) {
        ROS_WARN_STREAM("yaw: " << i << ": " << yawReader[i]);
        json_deserialized_msg["payload"]["discretizedPathPointsYaw"].push_back(yawReader[i]);
      }
    }

    if (msg_type != "ping") {
      std::string message_out =
          "Deserialized CONSOLE message[" + topic_name + "] " + json_deserialized_msg.dump();
      
      ROS_INFO_STREAM_COLOR(CONSOLE_MSG_COLOR, message_out);
    }
    
    //Rejections
    //Check vehicle_control_mode , if it is not in auto, then reject commands 
    if (msg_type != "ping" && msg_type != "linkup" && msg_type != "breaklink" && msg_type != "teardown")
    {
      if (vehicle_control_mode_.load() != 0) //NOT IN AUTO MODE, REJECT COMMANDS 
      {
         
         if (msg_type!="cmd_horn" && msg_type!="emergency_brake_command" && msg_type!="cmd_signal_light" &&
             msg_type != "cmd_head_light_remote")
         {
          ROS_WARN_STREAM("Vehicle Control Mode NOT in AUTO mode- Rejecting the teleop command");
          reject_reason = "Vehicle is NOT in AUTO mode. Rejecting the teleop command";
          rejected = true;
          return true;
         }
      }

      if (msg_type == "manual_push_remote")  //Nudge forward
      {
          if (!(session_job_status_.load() == 7 || session_job_status_.load() == 9)) //if there is no job in progress
          {
             ROS_WARN_STREAM(" [NUDGE] REJECTED: No Job in progress. Job Status:"<<session_job_status_.load());
             reject_reason = "There is no job In Progress";
             rejected = true;
             return true;
          }  
      }
      //we should grey out the buttons
      // if (msg_type == "adjust_position")  //Move By Distance
      // {
      //     if (session_job_status_.load() != 13) //alignment mode
      //     {
      //        ROS_WARN_STREAM(" [Move By Distance] REJECTED: APM is NOT in Alignment Mode!!");
      //        reject_reason = "APM is NOT in Alignment Mode";
      //        rejected = true;
      //        return true;
      //     }  
      // }

    }

    return true;
  } 
  catch (const kj::Exception& e) {
    // KJ_LOG(ERROR, "ERROR - deserializeMsgFromConsole [kj::Exception]:", e);
    std::cerr<<"1-ERROR - deserializeMsgFromConsole[kj::Exception]: " << topic_name<<"-"<<e.getDescription().cStr()<<std::endl;
    errorRaised = true;
    return false;
  }
  catch (const std::exception& e) {
    std::cerr<<"2-ERROR - deserializeMsgFromConsole [std::exception]:" << topic_name<<"-"<<e.what()<<std::endl;
    errorRaised = true;
    return false;
  }
  catch (nlohmann::json::type_error& e) {
    std::cerr << "3-ERROR - deserializeMsgFromConsole [type_error]:" << topic_name<<"-"<<e.what() << std::endl;
    errorRaised = true;
    return false;
  }
  catch (const std::logic_error& e) {
    std::cerr << "4-ERROR - deserializeMsgFromConsole [logic_error]:" << topic_name<<"-"<<e.what() << std::endl;
    errorRaised = true;
    return false;
  }
  catch (const nlohmann::json::parse_error& e) {
    std::cerr << "5-ERROR - deserializeMsgFromConsole [parse_error]:" << topic_name<<"-"<<e.what() << std::endl;
    errorRaised = true;
    return false;
  }
};

void RemoteOps::deserializeMsgFromVehicle(mqtt::const_message_ptr msg,
                                          nlohmann::json& json_deserialized_msg) {
  
 
  auto serializedMsg = msg->get_payload_str();
  std::string topic_name = msg->get_topic();
  try
  {
    kj::ArrayPtr<const capnp::word> capnpArray(
        reinterpret_cast<const capnp::word*>(serializedMsg.data()),
        serializedMsg.size() / sizeof(capnp::word));

    capnp::FlatArrayMessageReader array_message_reader(capnpArray);
    // Access the root of message
    Message::Reader message_reader = array_message_reader.getRoot<Message>();

    json_deserialized_msg["timestamp"] = message_reader.getTimestamp();
    json_deserialized_msg["type"] = message_reader.getType().cStr();
    json_deserialized_msg["version"] = message_reader.getVersion();
    json_deserialized_msg["payloadSize"] = message_reader.getUncompressedPayloadSize();

    // Access the root of payload
    auto payload_reader = message_reader.getPayload();

    // Obtain the pointer to the underlying data and its size
    const auto* payload_ptr = payload_reader.asBytes().begin();
    const size_t payload_size = payload_reader.asBytes().size() / sizeof(capnp::word);
    // Create a kj::ArrayPtr<const capnp::word> from the underlying data
    kj::ArrayPtr<const capnp::word> payload_array_ptr(
        reinterpret_cast<const capnp::word*>(payload_ptr), payload_size);
    capnp::FlatArrayMessageReader payload_msg_reader(payload_array_ptr);

    // std::cout << "\ttimestamp:" << json_deserialized_msg["timestamp"] << std::endl;
    // std::cout << "\ttype: " << json_deserialized_msg["type"] << std::endl;
    // std::cout << "\tversion: " << json_deserialized_msg["version"] << std::endl;
    // std::cout << "\tpayload size: " << json_deserialized_msg["payloadSize"] << std::endl;

    std::string msg_type = "";
    std::size_t found = topic_name.rfind('/');

    if (found != std::string::npos) {
      msg_type = topic_name.substr(found + 1);
    }

    if (msg_type == "linkup") {
      LinkUp::Reader linkup_reader = payload_msg_reader.getRoot<LinkUp>();
      std::string console_id =
          linkup_reader.getCommand().getLinkUpFromConsoleAck().getConsoleId().cStr();
      std::string vehicle_id =
          linkup_reader.getCommand().getLinkUpFromConsoleAck().getVehicleId().cStr();
      auto view_type = linkup_reader.getCommand().getLinkUpFromConsoleAck().getViewType();
      auto linkup_result = linkup_reader.getCommand().getLinkUpFromConsoleAck().getResult();
      json_deserialized_msg["consoleID"] = console_id;
      json_deserialized_msg["vehicleID"] = vehicle_id;
      json_deserialized_msg["viewType"] = view_type;
      json_deserialized_msg["result"] = linkup_result;

      if (linkup_result == LinkUpResult::SUCCESS) {
        std::cout << "\t******Linkup Successful..******" << std::endl;

      } else {
        std::cout << "\t******Linkup Failed..******" << std::endl;
      }
    } else if (msg_type == "breaklink") {
      BreakLink::Reader breaklink_reader = payload_msg_reader.getRoot<BreakLink>();

      if (breaklink_reader.getCommand().hasBreakLinkFromConsole())  // breaklink request
      {
        std::string console_id =
            breaklink_reader.getCommand().getBreakLinkFromConsole().getConsoleId().cStr();
        std::string vehicle_id =
            breaklink_reader.getCommand().getBreakLinkFromConsole().getVehicleId().cStr();

        BreakLinkReason breaklink_reason =
            breaklink_reader.getCommand().getBreakLinkFromConsole().getBreakLinkReason();

        json_deserialized_msg["consoleID"] = console_id;
        json_deserialized_msg["vehicleID"] = vehicle_id;
        json_deserialized_msg["breakLinkReason"] = breaklink_reason;

      } else  // breaklink ack
      {
        std::string console_id =
            breaklink_reader.getCommand().getBreakLinkFromConsoleAck().getConsoleId().cStr();
        std::string vehicle_id =
            breaklink_reader.getCommand().getBreakLinkFromConsoleAck().getVehicleId().cStr();
        json_deserialized_msg["consoleID"] = console_id;
        json_deserialized_msg["vehicleID"] = vehicle_id;
      }

    } else if (msg_type == "teardown") {
      Teardown::Reader teardown_reader = payload_msg_reader.getRoot<Teardown>();

      std::string console_id =
          teardown_reader.getCommand().getTeardownFromConsoleAck().getConsoleId().cStr();
      std::string vehicle_id =
          teardown_reader.getCommand().getTeardownFromConsoleAck().getVehicleId().cStr();
      TeardownResult teardown_result =
          teardown_reader.getCommand().getTeardownFromConsoleAck().getResult();

      // std::cout << "\tPayload [" << topic_name << "]" << std::endl;
      // std::cout << "\t  Console ID: " << console_id << std::endl;
      // std::cout << "\t  Vehicle ID: " << vehicle_id << std::endl;
      // std::cout << "\t  Result: " << getTeardownResult(teardown_result) << std::endl;

      json_deserialized_msg["consoleID"] = console_id;
      json_deserialized_msg["vehicleID"] = vehicle_id;
      json_deserialized_msg["result"] = teardown_result;

      if (teardown_result == TeardownResult::SUCCESS) {
        std::cout << "\t******Teardown Successful..******" << std::endl;
        // Do some action
      } else {
        std::cout << "\t******Teardown Failed..******" << std::endl;
        // Do some action
      }
    } else if (msg_type == "ping") {
      PingMessage::Reader ping_message_reader = payload_msg_reader.getRoot<PingMessage>();

      std::string sender = ping_message_reader.getSender().cStr();
      std::string receiver = ping_message_reader.getReceiver().cStr();
      uint8_t seq = ping_message_reader.getSeq();

      json_deserialized_msg["sender"] = sender;
      json_deserialized_msg["receiver"] = receiver;
      json_deserialized_msg["seq"] = seq;
      if (ping_message_reader.getType().hasPingAck()) {
        auto latency = ping_message_reader.getType().getPingAck().getRequestLatency();
        json_deserialized_msg["latency"] = latency;
      }
    } else if (msg_type == "ego_state") {
      EgoState::Reader ego_state_message_reader = payload_msg_reader.getRoot<EgoState>();
      json_deserialized_msg["signalLight"] = ego_state_message_reader.getSignalLight();
      json_deserialized_msg["vehPositionX"] = ego_state_message_reader.getVehPositionX();
      json_deserialized_msg["vehPositionY"] = ego_state_message_reader.getVehPositionY();
      json_deserialized_msg["vehYaw"] = ego_state_message_reader.getVehYaw();
      json_deserialized_msg["trailerPositionX"] = ego_state_message_reader.getTrailerPositionX();
      json_deserialized_msg["trailerPositionY"] = ego_state_message_reader.getTrailerPositionY();
      json_deserialized_msg["trailerYaw"] = ego_state_message_reader.getTrailerYaw();
      json_deserialized_msg["gearStatus"] = ego_state_message_reader.getGearStatus();
      json_deserialized_msg["brakePercentage"] = ego_state_message_reader.getParkingBrakeStatus();
      json_deserialized_msg["velocity"] = ego_state_message_reader.getVelocity();
      json_deserialized_msg["steeringAngle"] = ego_state_message_reader.getSteeringAngle();
      json_deserialized_msg["remoteEmergencyButtonStatus"] =
          ego_state_message_reader.getRemoteEmergencyButtonStatus();
      json_deserialized_msg["localizationStatus"] = ego_state_message_reader.getLocalizationStatus();
      json_deserialized_msg["headLights"] = ego_state_message_reader.getHeadLights();
      json_deserialized_msg["throttlePercentage"] = ego_state_message_reader.getThrottlePercentage();
      json_deserialized_msg["hornCmdFeedback"] = ego_state_message_reader.getHornCmdFeedback();
      json_deserialized_msg["inSSA"] = ego_state_message_reader.getInSSA();
    }

    bool display_msg = true;
    if (msg_type == "ping" && !debug_) {
      display_msg = false;
    }
    if (display_msg) {
      std::string message_out =
          "Deserialized [" + topic_name + "] FROM Vehicle: " + json_deserialized_msg.dump();
      ROS_INFO_STREAM_COLOR(VEHICLE_MSG_COLOR, message_out);
    }
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "1-deserializeMsgFromVehicle:Exception[type_error]: " << topic_name<<"-"<<e.what() << std::endl;
  }
  catch (const std::exception& e) {
      std::cerr<<"2-deserializeMsgFromVehicle:Exception[std]: " << topic_name<<"-"<<e.what() << std::endl;
  }
  catch (const kj::Exception& e) {
      std::cerr<<"3-deserializeMsgFromVehicle: Exception[kj::Exception]: " << topic_name<<"-" << e.getDescription().cStr()<<std::endl;
  }
  catch (const std::logic_error& e) {
      std::cerr<<"4-deserializeMsgFromVehicle: Exception[logic_error]: " << topic_name<<"-"<<e.what() << std::endl;
  }
  catch (const nlohmann::json::parse_error& e) {
      std::cerr<<"5-deserializeMsgFromVehicle: Exception[parse error]: " << topic_name<<"-"<<e.what() << std::endl;
  }
};

nlohmann::json RemoteOps::markerArrayToJson(MarkerArray::Reader& marker_list) {
  
    
  nlohmann::json result = nlohmann::json::array();
  try{  
    auto markers = marker_list.getMarkers();
    
    for (auto marker : markers) {
        result.push_back(markerToJson(marker));
    }
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "1-markerArrayToJson:Exception[type_error]: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
      std::cerr<<"2-markerArrayToJson:Exception[std]: " << e.what() << std::endl;
  }
  catch (const std::logic_error& e) {
      std::cerr<<"4-markerArrayToJson: Exception[logic_error]: " << e.what() << std::endl;
  }
  catch (const nlohmann::json::parse_error& e) {
      std::cerr<<"5-markerArrayToJson: Exception[parse error]: " << e.what() << std::endl;
  }
  return result;
}

nlohmann::json RemoteOps::markerToJson(Marker::Reader& marker) {
    nlohmann::json j;
    try
    {
      j["stamp"] = marker.getStamp();
      j["ns"] = marker.getNs();
      j["text"] = marker.getText();
      j["id"] = marker.getId();
      j["type"] = marker.getType();
      j["action"] = marker.getAction();

      // Convert pose (position + orientation)
      j["pose"]["position"]["x"] = marker.getPositionX();
      j["pose"]["position"]["y"] = marker.getPositionY();
      j["pose"]["position"]["z"] = marker.getPositionZ();;
      j["pose"]["orientation"]["x"] = marker.getOrientationX();
      j["pose"]["orientation"]["y"] = marker.getOrientationY();
      j["pose"]["orientation"]["z"] = marker.getOrientationZ();
      j["pose"]["orientation"]["w"] = marker.getOrientationW();

      // Convert scale
      j["scale"]["x"] = marker.getScaleX();
      j["scale"]["y"] = marker.getScaleY();
      j["scale"]["z"] = marker.getScaleZ();

      // Convert color
      j["color"]["r"] = marker.getColorR();
      j["color"]["g"] = marker.getColorG();
      j["color"]["b"] = marker.getColorB();
      j["color"]["a"] = marker.getColorA();
    
      
      auto points = marker.getPoints();
      std::vector<float> vec_points;
      for (auto p : points) {
          vec_points.push_back(p);
      }
      j["points"] = vec_points;
    }
    catch (nlohmann::json::type_error& e) {
        std::cerr << "1-markerToJson:Exception[type_error]: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
      std::cerr<<"2-markerToJson:Exception[std]: " << e.what() << std::endl;
    }
    catch (const std::logic_error& e) {
      std::cerr<<"4-markerToJson: Exception[logic_error]: " << e.what() << std::endl;
    }
    catch (const nlohmann::json::parse_error& e) {
      std::cerr<<"5-markerToJson: Exception[parse error]: " << e.what() << std::endl;
    }
    return j;
}


nlohmann::json RemoteOps::decodeMessage(std::string serializedMsg, std::string topic_name) {
  nlohmann::json json_deserialized_msg;
  try
  {
    kj::ArrayPtr<const capnp::word> capnpArray(
        reinterpret_cast<const capnp::word*>(serializedMsg.data()),
        serializedMsg.size() / sizeof(capnp::word));
    
    capnp::FlatArrayMessageReader array_message_reader(capnpArray);
    // Access the root of message
    Message::Reader message_reader = array_message_reader.getRoot<Message>();

    json_deserialized_msg["timestamp"] = message_reader.getTimestamp();
    json_deserialized_msg["type"] = message_reader.getType().cStr();
    json_deserialized_msg["version"] = message_reader.getVersion();
    json_deserialized_msg["payloadSize"] = message_reader.getUncompressedPayloadSize();

    std::string msg_type = "";
    std::size_t found = topic_name.rfind('/');

    if (found != std::string::npos) {
      msg_type = topic_name.substr(found + 1);
    }

    // Access the root of payload
    auto payload_reader = message_reader.getPayload();


    // Obtain the pointer to the underlying data and its size
    const auto* payload_ptr = payload_reader.asBytes().begin();
    const size_t payload_size = payload_reader.asBytes().size() / sizeof(capnp::word);
  
  
  
  //Visualization topics

    if (msg_type == "ehmi_info" )
    {
      std::string ehmi_info(payload_reader.asBytes().begin(),payload_reader.asBytes().end());
      json_deserialized_msg["payload"]["ehmiInfo"] = ehmi_info;
      
     
      return json_deserialized_msg;
    }
    else if (msg_type == "avcs_destination_from_ui" )
    {
      std::string destination(payload_reader.asBytes().begin(),payload_reader.asBytes().end());
      json_deserialized_msg["payload"]["destination"] = destination;
      return json_deserialized_msg;
    }
    else if (msg_type == "avcs_job_status")
    {
        // Convert the Data to int32_t
      // if (json_deserialized_msg["payloadSize"] == sizeof(int32_t)) {
          // int32_t intValue;
          // memcpy(&intValue, payload_ptr, sizeof(int32_t));
          
          // // Use the deserialized integer
          // std::cout << "Deserialized int32_t value: " << intValue << std::endl;
          // json_deserialized_msg["payload"]["jobStatus"] = intValue;
          std::string jobStatus(payload_reader.asBytes().begin(),payload_reader.asBytes().end());
          json_deserialized_msg["payload"]["jobStatus"] = jobStatus;
          return json_deserialized_msg;
      // } else {
      //     std::cerr << "Error: Data size does not match size of int32_t." << std::endl;
      //     json_deserialized_msg["payload"]["jobStatus"] = "";
      //     return json_deserialized_msg;
      // }
    }
    // Create a kj::ArrayPtr<const capnp::word> from the underlying data
    kj::ArrayPtr<const capnp::word> payload_array_ptr(
        reinterpret_cast<const capnp::word*>(payload_ptr), payload_size);
    capnp::FlatArrayMessageReader payload_msg_reader(payload_array_ptr);


    if (msg_type == "linkup") {
      LinkUp::Reader linkup_reader = payload_msg_reader.getRoot<LinkUp>();
      std::string console_id =
          linkup_reader.getCommand().getLinkUpFromConsoleAck().getConsoleId().cStr();
      std::string vehicle_id =
          linkup_reader.getCommand().getLinkUpFromConsoleAck().getVehicleId().cStr();
      auto view_type = linkup_reader.getCommand().getLinkUpFromConsoleAck().getViewType();
      auto linkup_result = linkup_reader.getCommand().getLinkUpFromConsoleAck().getResult();
      json_deserialized_msg["payload"]["consoleID"] = console_id;
      json_deserialized_msg["payload"]["vehicleID"] = vehicle_id;
      json_deserialized_msg["payload"]["viewType"] = view_type;
      json_deserialized_msg["payload"]["result"] = linkup_result;

      if (linkup_result == LinkUpResult::SUCCESS) {
        ROS_WARN_STREAM("\t******LINKUP SUCCESSFUL******");

      } else {
        ROS_WARN_STREAM("\t******LINKUP FAILED******");
      }
    } else if (msg_type == "breaklink") {
      BreakLink::Reader breaklink_reader = payload_msg_reader.getRoot<BreakLink>();

      if (breaklink_reader.getCommand().hasBreakLinkFromConsoleAck())  // response
      {
        std::string console_id =
            breaklink_reader.getCommand().getBreakLinkFromConsoleAck().getConsoleId().cStr();
        std::string vehicle_id =
            breaklink_reader.getCommand().getBreakLinkFromConsoleAck().getVehicleId().cStr();
        json_deserialized_msg["payload"]["consoleID"] = console_id;
        json_deserialized_msg["payload"]["vehicleID"] = vehicle_id;
      } else  // request
      {
        std::string console_id =
            breaklink_reader.getCommand().getBreakLinkFromConsole().getConsoleId().cStr();
        std::string vehicle_id =
            breaklink_reader.getCommand().getBreakLinkFromConsole().getVehicleId().cStr();
        BreakLinkReason breaklink_reason =
            breaklink_reader.getCommand().getBreakLinkFromConsole().getBreakLinkReason();
        json_deserialized_msg["payload"]["consoleID"] = console_id;
        json_deserialized_msg["payload"]["vehicleID"] = vehicle_id;
        json_deserialized_msg["payload"]["breakLinkReason"] = breaklink_reason;
      }

    } else if (msg_type == "teardown") {
      Teardown::Reader teardown_reader = payload_msg_reader.getRoot<Teardown>();

      std::string console_id =
          teardown_reader.getCommand().getTeardownFromConsoleAck().getConsoleId().cStr();
      std::string vehicle_id =
          teardown_reader.getCommand().getTeardownFromConsoleAck().getVehicleId().cStr();
      TeardownResult teardown_result =
          teardown_reader.getCommand().getTeardownFromConsoleAck().getResult();

      json_deserialized_msg["payload"]["consoleID"] = console_id;
      json_deserialized_msg["payload"]["vehicleID"] = vehicle_id;
      json_deserialized_msg["payload"]["result"] = teardown_result;

      if (teardown_result == TeardownResult::SUCCESS) {
        ROS_WARN_STREAM("\t******TEARDOWN SUCCESSFUL******");
        // Do some action
      } else {
        ROS_WARN_STREAM("\t******TEARDOWN FAILED******");
        // Do some action
      }
    } else if (msg_type == "ping") {
      PingMessage::Reader ping_reader = payload_msg_reader.getRoot<PingMessage>();

      std::string sender = ping_reader.getSender().cStr();
      std::string receiver = ping_reader.getReceiver().cStr();
      auto seq = ping_reader.getSeq();
      json_deserialized_msg["payload"]["sender"] = sender;
      json_deserialized_msg["payload"]["receiver"] = receiver;
      json_deserialized_msg["payload"]["seq"] = seq;

      if (ping_reader.getType().hasPingAck()) {
        auto latency = ping_reader.getType().getPingAck().getRequestLatency();
        json_deserialized_msg["payload"]["requestLatency"] = latency;
      }

    } else if (msg_type == "ego_state") {
      EgoState::Reader ego_state_message_reader = payload_msg_reader.getRoot<EgoState>();
      json_deserialized_msg["payload"]["signalLight"] = ego_state_message_reader.getSignalLight();
      json_deserialized_msg["payload"]["vehPositionX"] = ego_state_message_reader.getVehPositionX();
      json_deserialized_msg["payload"]["vehPositionY"] = ego_state_message_reader.getVehPositionY();
      json_deserialized_msg["payload"]["vehYaw"] = ego_state_message_reader.getVehYaw();
      json_deserialized_msg["payload"]["trailerPositionX"] =
          ego_state_message_reader.getTrailerPositionX();
      json_deserialized_msg["payload"]["trailerPositionY"] =
          ego_state_message_reader.getTrailerPositionY();
      json_deserialized_msg["payload"]["trailerYaw"] = ego_state_message_reader.getTrailerYaw();
      json_deserialized_msg["payload"]["gearStatus"] = ego_state_message_reader.getGearStatus();
      json_deserialized_msg["payload"]["brakePercentage"] =
          ego_state_message_reader.getParkingBrakeStatus();
      json_deserialized_msg["payload"]["velocity"] = ego_state_message_reader.getVelocity();
      json_deserialized_msg["payload"]["steeringAngle"] = ego_state_message_reader.getSteeringAngle();
      json_deserialized_msg["payload"]["remoteEmergencyButtonStatus"] =
          ego_state_message_reader.getRemoteEmergencyButtonStatus();
      json_deserialized_msg["payload"]["localizationStatus"] =
          ego_state_message_reader.getLocalizationStatus();
      json_deserialized_msg["payload"]["headLights"] = ego_state_message_reader.getHeadLights();
      json_deserialized_msg["payload"]["throttlePercentage"] =
          ego_state_message_reader.getThrottlePercentage();
      json_deserialized_msg["payload"]["hornCmdFeedback"] =
          ego_state_message_reader.getHornCmdFeedback();
    } else if (msg_type == "vehicle_control_mode") {
      VehicleControlMode::Reader vcm_message_reader = payload_msg_reader.getRoot<VehicleControlMode>();
      json_deserialized_msg["payload"]["mode"] = vcm_message_reader.getMode();
      json_deserialized_msg["payload"]["pauseResume"] = vcm_message_reader.getPauseResume();

    } else if (msg_type == "traffic_light_status") {
      TrafficLight::Reader tl_message_reader = payload_msg_reader.getRoot<TrafficLight>();
      json_deserialized_msg["payload"]["color"] = tl_message_reader.getColor();
    } 
    else if (msg_type == "ops_response") {
    
      OpsResponse::Reader res_message_reader = payload_msg_reader.getRoot<OpsResponse>();
      json_deserialized_msg["payload"]["reqReceivedTime"] = res_message_reader.getReqReceivedTime();
      json_deserialized_msg["payload"]["operationStatus"] = res_message_reader.getOperationStatus();
      json_deserialized_msg["payload"]["operationType"] = res_message_reader.getOperationType();
      json_deserialized_msg["payload"]["reason"] = res_message_reader.getReason();

    } 
    else if (msg_type == "tf") {
    
      Transform::Reader tf_message_reader = payload_msg_reader.getRoot<Transform>();
      json_deserialized_msg["payload"]["stamp"] = tf_message_reader.getStamp();
      json_deserialized_msg["payload"]["translationX"] = tf_message_reader.getTranslationX();
      json_deserialized_msg["payload"]["translationY"] = tf_message_reader.getTranslationY();
      json_deserialized_msg["payload"]["translationZ"] = tf_message_reader.getTranslationZ();

      json_deserialized_msg["payload"]["rotationX"] = tf_message_reader.getRotationX();
      json_deserialized_msg["payload"]["rotationY"] = tf_message_reader.getRotationY();
      json_deserialized_msg["payload"]["rotationZ"] = tf_message_reader.getRotationZ();
      json_deserialized_msg["payload"]["rotationW"] = tf_message_reader.getRotationW();
    
      json_deserialized_msg["payload"]["frameId"] = tf_message_reader.getFrameId();
      json_deserialized_msg["payload"]["childFrameId"] = tf_message_reader.getChildFrameId();

    } else if (msg_type == "route_plan") {
    
      Path::Reader path_message_reader = payload_msg_reader.getRoot<Path>();
      json_deserialized_msg["payload"]["timestamp"] = path_message_reader.getTimestamp();
      
      ::capnp::List<uint32_t>::Reader  segment_id = path_message_reader.getSegmentId();
      ::capnp::List<float>::Reader  points_x = path_message_reader.getDiscretizedPathPointsX();
      ::capnp::List<float>::Reader  points_y = path_message_reader.getDiscretizedPathPointsY();
      ::capnp::List<float>::Reader  points_yaw = path_message_reader.getDiscretizedPathPointsYaw();
      for (auto s: segment_id) {
          json_deserialized_msg["payload"]["segmentId"].push_back(s);
      }
      
      for (auto p_x: points_x) {
          json_deserialized_msg["payload"]["discretizedPathPointsX"].push_back(p_x);
      }
      for (auto p_y: points_y) {
          json_deserialized_msg["payload"]["discretizedPathPointsY"].push_back(p_y);
      }
      for (auto p_yaw: points_yaw) {
          json_deserialized_msg["payload"]["discretizedPathPointsYaw"].push_back(p_yaw);
      }

    } 
    else if (msg_type == "predicted_footprint") {
    
      Polygon::Reader polygon_message_reader = payload_msg_reader.getRoot<Polygon>();
      json_deserialized_msg["payload"]["stamp"] = polygon_message_reader.getStamp();
      json_deserialized_msg["payload"]["includesZ"] = polygon_message_reader.getIncludesZ();
      
      ::capnp::List<float>::Reader  points = polygon_message_reader.getPayload();
  
      for (auto p: points) {
          json_deserialized_msg["payload"]["points"].push_back(p);
      }
    } else if ( (msg_type == "vertical_points") || 
                (msg_type == "on_road_pc") ||  
                (msg_type == "curb_pc") || 
                (msg_type == "most_constrained_points") ||
                (msg_type == "road_intensity_detection")
              )
                
    
    {
      PCD::Reader pcd_message_reader = payload_msg_reader.getRoot<PCD>();
      json_deserialized_msg["payload"]["stamp"] = pcd_message_reader.getStamp();
      json_deserialized_msg["payload"]["includesZ"] = pcd_message_reader.getIncludesZ();
      json_deserialized_msg["payload"]["includesIntensity"] = pcd_message_reader.getIncludesIntensity();

    } else if (msg_type == "most_constrained_object") 
    {
      Marker::Reader marker_reader = payload_msg_reader.getRoot<Marker>();
      nlohmann::json json_obj = markerToJson(marker_reader);
      json_deserialized_msg["payload"] = json_obj;
    } else if (msg_type =="objects" || 
               msg_type =="crosswalk_vis" ||
               msg_type =="traffic_jam_lanes_vis" ||
               msg_type =="road_global_vis" || msg_type =="objects_of_interest" ||
               msg_type =="hatch_cover_detection_box" || msg_type =="rtg_detection_box" ) 
    {
      MarkerArray::Reader marker_array_reader = payload_msg_reader.getRoot<MarkerArray>();
      nlohmann::json json_obj = markerArrayToJson(marker_array_reader);
      json_deserialized_msg["payload"] = json_obj;
    }

    else if (msg_type == "remote_ops_request") {
   
      RemoteOpsRequest::Reader req_reader = payload_msg_reader.getRoot<RemoteOpsRequest>();
      ::capnp::List<uint32_t>::Reader  redAlerts = req_reader.getRedAlerts();
      ::capnp::List<uint32_t>::Reader  redActionableAlerts = req_reader.getRedActionableAlerts();
      ::capnp::List<uint32_t>::Reader  amberAlerts = req_reader.getAmberAlerts();
      ::capnp::List<uint32_t>::Reader  amberActionableAlerts = req_reader.getAmberActionableAlerts();
      ::capnp::List<uint8_t>::Reader   recommendedActions = req_reader.getRecommendedActions();
      ::capnp::List<uint8_t>::Reader   restrictedActions = req_reader.getRestrictedActions();
      ::capnp::List<uint8_t>::Reader   highRiskType = req_reader.getHighRiskType();
      ::capnp::List<float>::Reader     highRiskDist = req_reader.getHighRiskDist();
      ::capnp::List<uint32_t>::Reader   metricTypes = req_reader.getMetricTypes();
      ::capnp::List<capnp::Text>::Reader   metricValues = req_reader.getMetricValues();
      
      json_deserialized_msg["payload"]["redAlerts"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["redActionableAlerts"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["amberAlerts"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["amberActionableAlerts"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["recommendedActions"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["restrictedActions"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["highRiskType"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["highRiskDist"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["metricTypes"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["metricValues"] = nlohmann::json::array();
   

      for (auto el: redAlerts) {
          json_deserialized_msg["payload"]["redAlerts"].push_back(el);
      }

      for (auto el: redActionableAlerts) {
          json_deserialized_msg["payload"]["redActionableAlerts"].push_back(el);
      }

      for (auto el: amberAlerts) {
          json_deserialized_msg["payload"]["amberAlerts"].push_back(el);
      }

      for (auto el: amberActionableAlerts) {
          json_deserialized_msg["payload"]["amberActionableAlerts"].push_back(el);
      }
      for (auto el: recommendedActions) {
          json_deserialized_msg["payload"]["recommendedActions"].push_back(el);
      }
     
     
      
      for (auto el: restrictedActions) {
          json_deserialized_msg["payload"]["restrictedActions"].push_back(el);
      }
     
      for (auto el: highRiskType) {
          json_deserialized_msg["payload"]["highRiskType"].push_back(el);
      }
      for (auto el: highRiskDist) {
          json_deserialized_msg["payload"]["highRiskDist"].push_back(el);
      }
      for (auto el: metricTypes) {
          json_deserialized_msg["payload"]["metricTypes"].push_back(el);
      }
      for (auto el: metricValues) {
          auto text = el.cStr();
          json_deserialized_msg["payload"]["metricValues"].push_back(text);
      }
    
      bool job_done_allowed = req_reader.getJobDoneAllowed();
      json_deserialized_msg["payload"]["jobDoneAllowed"] = job_done_allowed;
      
    }


    else if (msg_type == "remote_ops_init") {
      RemoteOpsInit::Reader ops_init_reader = payload_msg_reader.getRoot<RemoteOpsInit>();
      
      ::capnp::List<uint8_t>::Reader   metricsType = ops_init_reader.getMetricsType();
      ::capnp::List<uint8_t>::Reader   metricsValueType = ops_init_reader.getMetricsValueType();
      ::capnp::List<uint32_t>::Reader  alertType = ops_init_reader.getAlertType();
      ::capnp::List<capnp::Text>::Reader alertThreshold = ops_init_reader.getAlertThreshold();
      ::capnp::List<uint32_t>::Reader  alertIds = ops_init_reader.getAlertIds();
      ::capnp::List<capnp::Text>::Reader alertDescriptions = ops_init_reader.getAlertDescriptions();
    
      json_deserialized_msg["payload"]["metricsType"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["metricsValueType"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["alertType"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["alertThreshold"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["alertIds"] = nlohmann::json::array();
      json_deserialized_msg["payload"]["alertDescriptions"] = nlohmann::json::array();
    

      for (auto el: metricsType) {
          json_deserialized_msg["payload"]["metricsType"].push_back(el);
      }

      for (auto el: metricsValueType) {
          json_deserialized_msg["payload"]["metricsValueType"].push_back(el);
      }

      for (auto el: alertType) {
          json_deserialized_msg["payload"]["alertType"].push_back(el);
      }

      for (auto el: alertThreshold) {
          auto text = el.cStr();
          json_deserialized_msg["payload"]["alertThreshold"].push_back(text);
      }

      for (auto el: alertIds) {
          json_deserialized_msg["payload"]["alertIds"].push_back(el);
      }

      for (auto el: alertDescriptions) {
          auto text = el.cStr();
          json_deserialized_msg["payload"]["alertDescriptions"].push_back(text);
      }
    }
 
  }
  catch (nlohmann::json::type_error& e) {
        std::cerr << "1-decodeMessage: Exception[type_error]:" << topic_name<<"-"<<e.what() << std::endl;
  }
  catch (const std::exception& e) {
      std::cerr<<"2-decodeMessage: Exception[std]"<< topic_name<<"-"<< e.what() << std::endl;
  }  
  catch (const kj::Exception& e) {
      std::cerr<<"3-decodeMessage: Exception[kj::Exception]:"<< topic_name<<"-" << e.getDescription().cStr()<<std::endl;
  } 
  catch (const std::logic_error& e) {
        std::cerr << "4-decodeMessage-Exception[logic_error]:" << e.what() << std::endl;
  }   
  catch (const nlohmann::json::parse_error& e) 
  {
        std::cerr << "5-decodeMessage-Exception[parse error]:"<< e.what() << std::endl;
  }  
  return json_deserialized_msg;
}

std::string RemoteOps::encodeMessage(std::string type, capnp::MallocMessageBuilder& payload,
                                     uint64_t& msg_timestamp, bool& errorRaised) {
  // Serialize message builder into a capnp::Data:Reader
  try
  {
      kj::VectorOutputStream outputStream;
      capnp::writeMessage(outputStream, payload);
      auto payloadSerializedData = outputStream.getArray();

      capnp::Data::Reader payloadData(
          reinterpret_cast<const capnp::byte*>(payloadSerializedData.begin()),
          payloadSerializedData.size());

      uint64_t ts = getTimestamp();
      msg_timestamp = ts;
      ::capnp::MallocMessageBuilder msg_builder;
      Message::Builder message = msg_builder.initRoot<Message>();

      uint16_t version = 1;
      message.setTimestamp(ts);
      message.setVersion(version);
      message.setType(type);
      message.setPayload(payloadData);
      message.setUncompressedPayloadSize(payloadSerializedData.size());

      // std::cout<<"Data Payload size:"<<payloadSerializedData.size();

      kj::Array<capnp::word> packed_data = ::capnp::messageToFlatArray(msg_builder);
      // ROS_WARN_STREAM("Serialized size:"<<packed_data.size());
      // Convert the packed data to a string for publishing
      std::string serializedString(reinterpret_cast<const char*>(packed_data.begin()),
                                  packed_data.size() * sizeof(capnp::word));

      if (type == "/remoteops/remote_ops_init") 
      {
        ROS_WARN_STREAM(type<<"-Serialized size:"<<serializedString.size());
      
      }
      return serializedString;
  }
  catch (const kj::Exception& e) {
      std::cerr<<"1-ERROR -  [encodeMessage][kj::Exception]: " << e.getDescription().cStr()<<std::endl;
      errorRaised = true; 
      return "";
  }
  catch (const std::exception& e) {
      std::cerr<<"2-ERROR -   [encodeMessage][std]:" << e.what()<<std::endl;
      errorRaised = true; 
      return "";
  }  
  catch (nlohmann::json::type_error& e) {
       std::cerr<<"3-ERROR -   [encodeMessage][type_error]:" << e.what()<<std::endl;
      errorRaised = true; 
      return "";
  }
  catch (const std::logic_error& e) {
       std::cerr<<"4-ERROR -   [encodeMessage][logic_error]:" << e.what()<<std::endl;
      errorRaised = true; 
      return "";
  }

}

void RemoteOps::writeMsgToFile(std::string file_name, std::string& msg, std::string sub_folder,
                               bool append_data) {
  std::string directory = log_folder_ + sub_folder + "/";
  bool err = false;
  try {
    if (!boost::filesystem::exists(directory)) {
      if (boost::filesystem::create_directories(directory)) {
        std::cout << "[writeMsgToFile] " << directory << " has been created." << std::endl;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "[writeMsgToFile] Exception caught: " << e.what() << std::endl;
    err = true;
  }

  if (!err) {
    std::string filepath = directory + file_name;
    // std::ofstream outputFile(filepath);
    std::ofstream outputFile;

    if (append_data) {
      outputFile.open(filepath, std::ios::app);
    } else {
      outputFile.open(filepath);  // Open "new_test.txt" in append mode
    }

    if (!outputFile.is_open()) {
      std::cerr << "[writeMsgToFile] Failed to open " + file_name + " file for writing!"
                << std::endl;
    }
    if (append_data) {
      msg = '\n' + msg;
    }
    outputFile << msg;
    outputFile.close();
  }
}

std::string RemoteOps::getHexPayload(const std::string& payload) {
  // Convert each byte of the payload to hexadecimal
  std::stringstream hex_payload_stream;
  hex_payload_stream << std::hex << std::setfill('0');
  for (char byte : payload) {
    hex_payload_stream << std::setw(2) << static_cast<int>(static_cast<unsigned char>(byte));
  }
  std::string hex_str = hex_payload_stream.str();
  return hex_str;
}

void RemoteOps::publishPayloadToROS(ros::Publisher& pub, nlohmann::json json_msg) {
  try
  {
    std_msgs::String pub_msg;
    pub_msg.data = json_msg.dump();
    pub.publish(pub_msg);

    std::string message_out = "Published payload to ROS:" + pub_msg.data;

    std::string color = "";
    std::string topic_name = json_msg["topic_name"];

    std::size_t found = topic_name.rfind('/');
    std::string msg_type = topic_name;
    if (found != std::string::npos) {
        msg_type = topic_name.substr(found + 1);
    }


    if (std::find(exclude_msg_types_from_logging_.begin(), exclude_msg_types_from_logging_.end(), msg_type) != exclude_msg_types_from_logging_.end()) {
     return;
    } 
    
    if (json_msg["sender"] == "vehicle") {
      color = VEHICLE_MSG_COLOR;
    } else {
      color = CONSOLE_MSG_COLOR;
    }
    ROS_INFO_STREAM_COLOR(color, message_out);
    std::string payload_file_name = log_folder_ + "payload.json";
    std::ofstream payload_file(payload_file_name, std::ios_base::app);
    payload_file << json_msg << "\n";
    payload_file.close();
  }

  catch (nlohmann::json::type_error& e) {
    std::cerr << "1-publishPayloadToROS Exception[type_error]: " <<e.what() << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "2-publishPayloadToROS Exception[std]: " <<e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
    std::cerr << "3-publishPayloadToROS Exception[logic_error]: " <<e.what() << std::endl;
  }
  catch (const nlohmann::json::parse_error& e) {
    std::cerr << "3-publishPayloadToROS Exception[parse_error]: " <<e.what() << std::endl;
  }
}

void RemoteOps::publishMQTTMessage(std::string msg_type, std::string& serializedString,
                                   std::string topic_name, int qos, std::string sender,
                                   bool isVehiclePingAck) {
  // Set up the message payload
  // ROS_WARN_STREAM("Publishing MQTT:"<<msg_type<< " QOS: "<<qos);
  mqtt::message_ptr msg = mqtt::make_message(topic_name, serializedString, 0, false);
  msg->set_qos(qos);
  // Publish the message

  try {
    auto start_time = std::chrono::high_resolution_clock::now();
    cli_->publish(msg)->wait_for(std::chrono::milliseconds(int(mqtt_pub_wait_sec_ * 1000)));

    // cli_->publish(topic_name, serializedString, qos, false);

    auto end_time = std::chrono::high_resolution_clock::now();
    int64_t latency =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    if (latency > publish_MQTT_latency_) {
      std::cout << "MQTT Client " << topic_name << " publish latency *Increasing:" << latency
                << std::endl;
    }
    // else
    // {
    //    std::cout<<"MQTT Client publish latency *Decreasing:"<<latency<<std::endl;

    // }
    publish_MQTT_latency_ = latency;

    // if (isVehiclePingAck)
    // {
    //   vehicle_ping_mqtt_pub_ts_ = getTimestamp();//mqtt published ts

    //   auto time_diff = (vehicle_ping_mqtt_pub_ts_ - console_ping_arrive_ts_);
    //   std::chrono::nanoseconds time_diff_ns(time_diff);
    //   std::chrono::milliseconds time_diff_ms =
    //             std::chrono::duration_cast<std::chrono::milliseconds>(time_diff_ns);

    //   vehicle_ping_process_duration_ = time_diff_ns.count();
    //   ROS_WARN_STREAM("Ping process time:"<<vehicle_ping_process_duration_);
    // }

    std::string color = "magenta";
    /**********************/
    if (sender == "vehicle") {
      if (msg_type == "linkup") {
        nlohmann::json deserialized_msg = decodeMessage(serializedString, topic_name);
        auto result = deserialized_msg["payload"]["result"];
        if (result == LinkUpResult::SUCCESS) {
          setLinkupStatus(true);
          setBreaklinkStatus(false);
          setTeardownStatus(false);
        }
      } else if (msg_type == "breaklink") {
        setLinkupStatus(false);
        setBreaklinkStatus(true);
        setTeardownStatus(false);
        std::string msg_out = "BreaklinkStatus:True";
        ROS_INFO_STREAM_COLOR("magenta", msg_out);
      } else if (msg_type == "teardown") {
        nlohmann::json deserialized_msg = decodeMessage(serializedString, topic_name);
        auto result = deserialized_msg["payload"]["result"];
        if (result == TeardownResult::SUCCESS) {
          setLinkupStatus(false);
          setBreaklinkStatus(false);
          setTeardownStatus(true);
          std::string msg_out = "TeardownStatus:True ";
          ROS_INFO_STREAM_COLOR("magenta", msg_out);
        }
      }
      
      if (std::find(exclude_msg_types_from_logging_.begin(), exclude_msg_types_from_logging_.end(), msg_type) != exclude_msg_types_from_logging_.end()) 
      {
        return;
      } 
      else 
      {
        nlohmann::json deserialized_msg = decodeMessage(serializedString, topic_name);

        nlohmann::json pub_msg;
        pub_msg["sender"] = "vehicle";
        pub_msg["timestamp"] = deserialized_msg["timestamp"];
        pub_msg["topic_name"] = topic_name;
        pub_msg["payload"] = deserialized_msg["payload"];

        std::string file_name = msg_type + "_" + to_string(deserialized_msg["timestamp"]) + ".txt";
        
        publishPayloadToROS(pub_outgoing_payload_, pub_msg);
        std::string hex_payload = getHexPayload(msg->to_string());
        std::string str_payload = msg->to_string();
         
        writeMsgToFile(file_name, str_payload, "outgoing_payload");
        
      }
    }

    // if ((msg_type == "consolePingAck") || (msg_type == "ping") || 
    //     (msg_type == "ego_state") || (msg_type == "tf")) || 

    // {
    //   return;
    // }

    if (msg_type == "consoleBreaklinkAck") {
      sender = "console";
    }

    std::string message_out = (sender + " published MQTT Message [" + topic_name + "]");
    ROS_INFO_STREAM_COLOR("magenta", message_out);

    /**********************/
    std::cout << std::endl << serializedString << std::endl;
    std::cout << "---------------------" << std::endl;

  } catch (const mqtt::exception& exc) {
    std::cerr << "1-publishMQTT Client Error[mqtt::exception]:" << exc << std::endl;
  }
  catch (nlohmann::json::type_error& e) {
    std::cerr << "2-publishMQTT Exception[type_error]: " <<e.what() << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "3-publishMQTT Exception[std]:  " <<e.what() << std::endl;
  }  
  catch (const nlohmann::json::parse_error& e) {
    std::cerr << "4-publishMQTT Exception[parse_error]:  " <<e.what() << std::endl;
  }  
  catch (const std::logic_error& e) {
    std::cerr << "5-publishMQTT Exception[logic_error]:  " <<e.what() << std::endl;
  }  
}
