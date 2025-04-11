#ifndef REMOTE_CONSOLE_H_
#define REMOTE_CONSOLE_H_
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <kj/array.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <boost/crc.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>

#include "mqtt/async_client.h"
#include <aios_lib/aios_enum_classes.h>
#include <aios_lib/psa_enum_classes.h>
// CAPNP Messages
#include <nlohmann/json.hpp>

#include "ac_control_cmd.capnp.h"
#include "adjust_position_by_distance_cmd.capnp.h"
#include "breaklink.capnp.h"
#include "complete_job_cmd.capnp.h"
#include "emergency_brake_cmd.capnp.h"
#include "fog_light_cmd.capnp.h"
#include "gear_cmd.capnp.h"
#include "hazard_light_cmd.capnp.h"
#include "head_light_cmd.capnp.h"
#include "horn_cmd.capnp.h"
#include "ignition_cmd.capnp.h"
#include "indicator_light_cmd.capnp.h"
#include "light_state_cmd.capnp.h"
#include "linkup.capnp.h"
#include "message.capnp.h"
#include "nudge_forward_cmd.capnp.h"
#include "pause_resume_cmd.capnp.h"
#include "path.capnp.h"
#include "ping.capnp.h"
#include "precedence_override_cmd.capnp.h"
#include "relocalization_cmd.capnp.h"
#include "remote_ops_cmd.capnp.h"
#include "set_destination_cmd.capnp.h"
#include "set_non_yard_destination_cmd.capnp.h"
#include "teardown.capnp.h"
#include "traffic_light_override_cmd.capnp.h"
#include "weather_state_cmd.capnp.h"
#include "working_light_cmd.capnp.h"
//Visualization
#include "ego_state.capnp.h"
#include "vehicle_control_mode.capnp.h"
#include "transform.capnp.h"
#include "traffic_light.capnp.h"
#include "ops_response.capnp.h"
#include "polygon.capnp.h"
#include "pcd.capnp.h"
#include "marker.capnp.h"
#include "marker_array.capnp.h"
#include "remote_ops_request.capnp.h"
#include "remote_ops_init.capnp.h"

typedef kj::ArrayPtr<kj::byte> kjByteArrayPtr;
typedef kj::ArrayPtr<capnp::word> kjWordArrayPtr;
const int C_QOS = 2;  // For command data
const int V_QOS = 0;  // For visualization data
const std::string VEHICLE_MSG_COLOR = "green";
const std::string CONSOLE_MSG_COLOR = "blue";

class RemoteOps {
 public:
  RemoteOps(ros::NodeHandle& private_nh, std::shared_ptr<mqtt::async_client> cli);

  uint32_t CRC32(const std::string& in);
  std::string currentDateTime();

  uint64_t getTimestamp();

  inline void setVehicleID(std::string vehicle_id) { vehicle_id_ = vehicle_id; };

  inline void setConsoleID(std::string console_id) { console_id_ = console_id; };

  inline void setViewType(std::string view_type) {
    view_type_ = ((view_type == "Control") ? ConsoleViewType::CONTROL : ConsoleViewType::VIEW);
  };

  inline void setConsoleMode(std::string console_mode) {
    console_mode_ = ((console_mode == "Test") ? ConsoleMode::TEST : ConsoleMode::PRODUCTION);
  };

  inline std::string getViewType(ConsoleViewType view_type) {
    return ((view_type == ConsoleViewType::CONTROL) ? "Control" : "View");
  };

  inline std::string getConsoleMode(ConsoleMode mode) {
    return ((mode == ConsoleMode::TEST) ? "Test" : "Production");
  };

  inline void setHeadLightState(int state) {
    //[0 - Head light off], [1 - Low beam], [2 - High beam]
    head_light_state_ = state;
  }

  inline void setIndicatorLightState(int state) { indicator_light_state_ = state; }
  inline void setFogLightState(bool state) { fog_light_state_ = state; }
  inline void setIgnitionState(bool state) { ignition_state_ = state; }
  inline void setAcState(bool state) { ac_state_ = state; }
  inline void setPrecedenceOverrideState(bool state) { precedence_override_state_ = state; }
  inline void setGearState(int state) { gear_change_state_ = state; }
  inline void setPauseResume(bool pause) { pause_ = pause; }
  inline void setWorkingLight(bool rear_working_light, bool front_working_light) {
    rear_working_light_ = rear_working_light;
    front_working_light_ = front_working_light;
  }
  inline void setTrafficLightOverrideState(int state) {
      traffic_light_override_state_ = state;
  }
  inline void setWeatherState(int state) { weather_state_ = state; }
  inline void setLightState(int state) { light_state_ = state; }

  void setBreaklinkReason(std::string breaklink_reason);
  std::string getBreaklinkReason(BreakLinkReason breaklink_reason);

  inline void setTeardownResult(TeardownResult teardown_result) {
    teardown_result_ = teardown_result;
  };

  std::string getTeardownResult(TeardownResult result);

  inline void setLinkupResult(LinkUpResult linkup_result) { linkup_result_ = linkup_result; };
  std::string getLinkupResult(LinkUpResult result);

  void serializeMsgFromConsole(std::string topic_name, std::string msg_type,  bool isEmergencyBrake = false, bool isHazardLights = false, uint8_t nudgeInstanceId = 0);
  void serializeMsgFromVehicle(nlohmann::json& payload, bool& errorRaised);

  bool deserializeMsgFromConsole(mqtt::const_message_ptr msg,
                                 nlohmann::json& json_deserialized_msg,
                                 bool& errorRaised, bool& rejected, std::string& reject_reason);
  void deserializeMsgFromVehicle(mqtt::const_message_ptr msg,
                                 nlohmann::json& json_deserialized_msg);

  std::string encodeMessage(std::string type, capnp::MallocMessageBuilder& message,
                            uint64_t& msg_timestamp, bool& errorRaised);
  nlohmann::json decodeMessage(std::string serializedMsg, std::string topic_name);
  void writeMsgToFile(std::string file_name, std::string& msg, std::string sub_folder,
                      bool append_data = false);
  void publishMQTTMessage(std::string msg_type, std::string& serializedString,
                          std::string topic_name, int qos, std::string sender, bool isVehiclePingAck = false);
  void publishPayloadToROS(ros::Publisher& pub, nlohmann::json json_msg);
  std::string getHexPayload(const std::string& payload);

  void ROS_INFO_STREAM_COLOR(const std::string& color, auto text);

  inline std::string getConsoleID() { return console_id_; }
  inline std::string getVehicleID() { return vehicle_id_; }
  inline bool getLinkupStatus() { return linkup_status_; }
  inline bool getBreaklinkStatus() { return breaklink_status_; }
  inline bool getTeardownStatus() { return teardown_status_; }
  
  inline void setLinkupStatus(bool status) { linkup_status_ = status; }
  inline void setBreaklinkStatus(bool status) { breaklink_status_ = status; }
  inline void setTeardownStatus(bool status) { teardown_status_ = status; }
  inline void setVehicleAckPingRequestLatency(uint64_t latency) {
    vehicle_ack_ping_request_latency_ = latency;
  }
  inline bool getVehicleBreakLinkRequest() { return isVehicleBreakLinkRequest_; }

  uint8_t getOperationType(std::string msg_type);

  nlohmann::json markerToJson(Marker::Reader& marker);
  nlohmann::json markerArrayToJson(MarkerArray::Reader& marker_list);
  
  bool debug_ = false;
  uint64_t console_ping_arrive_ts_, vehicle_ping_mqtt_pub_ts_;
  double vehicle_ping_process_duration_;
  std::vector<std::string> vec_visualization_topic_;
 private:
  ros::NodeHandle& private_nh_;
  ros::Publisher pub_outgoing_payload_;
  double mqtt_pub_wait_sec_;
  ::capnp::MallocMessageBuilder msg_builder_;
  Message::Builder message_ = msg_builder_.initRoot<Message>();
  ConsoleViewType view_type_;
  ConsoleMode console_mode_;
  BreakLinkReason breaklink_reason_;
  LinkUpResult linkup_result_;
  TeardownResult teardown_result_;

  std::string console_id_, vehicle_id_;

  int head_light_state_;
  int indicator_light_state_;
  int gear_change_state_;
  int traffic_light_override_state_ = 0;
  int weather_state_, light_state_;
  bool fog_light_state_;
  bool ignition_state_;
  bool rear_working_light_, front_working_light_;
  bool pause_;
  bool ac_state_;
  bool precedence_override_state_;

  std::shared_ptr<mqtt::async_client> cli_;
  std::string user_id_;
  std::string log_folder_;

  bool linkup_status_ = false;
  bool breaklink_status_ = false;
  bool teardown_status_ = false;

  uint8_t console_send_ping_seq_ = 0;
  uint64_t vehicle_ack_ping_request_latency_ = 0;
  bool isVehicleBreakLinkRequest_ = false;
  int64_t publish_MQTT_latency_ = -1;

  std::atomic<int> vehicle_control_mode_;
  std::atomic<int> avcs_job_status_;
  std::atomic<int> apm_status_;
  std::atomic<int> session_job_status_;

  std::vector<std::string>  exclude_msg_types_from_logging_ = {"ping", "tf", "ego_state", "consolePingAck",
                                                               "most_constrained_object",
                                                               "crosswalk_vis", "traffic_jam_lanes_vis",  
                                                               "road_global_vis",
                                                               "objects_of_interest",
                                                               "objects",
                                                               "hatch_cover_detection_box",
                                                               "rtg_detection_box", "most_constrained_points",
                                                               "vertical_points", 
                                                               "on_road_pc",
                                                               "curb_pc", "road_intensity_detection",
                                                               /*"predicted_footprint", */
                                                               /*"route_plan", */
                                                               "remote_ops_request",
                                                               "traffic_light_status", "consolePingAck", "ops_response", 
                                                               "vehicle_control_mode", "avcs_job_status" };

 
  
 
};

#endif
