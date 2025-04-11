#ifndef _REMOTE_HANDLER_HPP_
#define _REMOTE_HANDLER_HPP_

#include <aios_apm_msgs/AiosPreprocess.h>
#include <aios_apm_msgs/EgoState.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <boost/crc.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <aios_lib/aios_enum_classes.h>
#include <aios_lib/psa_enum_classes.h>
#include <aios_lib/transform.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath> 

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


class RemoteHandler {
 public:
  // public functions
  RemoteHandler(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void initializeMqttComms();
  void getParams();

  inline std::string getVehicleID() { return client_id_; };
  void publishToMqtt(nlohmann::json& root_msg);

  inline bool isBrokerConnected() { return broker_connected_; }
  inline bool isMqttInitialized() { return mqtt_initialized_; }

  void sendRemoteRequest(std::string msg_type);
  void handleRemoteConsoleRequest(const std::string& msg_type, const nlohmann::json& remote_response_msg);
  void publishTrafficLightStatus(int color);
  void publishAvcsDestinationFromUi(std::string dest);
  void publishAvcsJobStatus(int aios_job_status);
  void publishJobInfo();
  void publishRemoteOpsRequest();
  void setErrorReset();
  
  //CALLBACKS
  void aiosPreprocessCallback(const aios_apm_msgs::AiosPreprocess::ConstPtr& msg);
  void vehicleControlModeCallback(const std_msgs::Int16::ConstPtr& msg);
  void ehmiInfoCallback(const std_msgs::String::ConstPtr& msg);
  void localizationStatusCallback(const std_msgs::Float32::ConstPtr& msg); 
  void fromMQTTCallback(const std_msgs::String::ConstPtr& msg);

  void remoteOpsRequestCallback(const std_msgs::String::ConstPtr& msg);

  //Marker Callbacks
  void mostConstrainedObjectCallback(const visualization_msgs::Marker::ConstPtr& msg);
  void objectsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void crosswalkVisCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void trafficJamLanesVisCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void roadGlobalVisCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void objectsOfInterestCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void hatchCoverDetectionBoxCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void rtgDetectionBoxCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);

  //Polygon
  void predictedFootprintCallback(const geometry_msgs::Polygon::ConstPtr& msg);
  
  //Path
  void routePlanCallback(const geometry_msgs::Polygon::ConstPtr& msg);

   //PCD Callbacks
  void verticalPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void onRoadPCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void curbPCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void mostConstrainedPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void roadIntensityDetectionCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  
  nlohmann::json markerToJson(const visualization_msgs::Marker& marker);
  void parseJsonFile(std::string param_name);
  void publishRemoteOpsInit();
  void publishLinkupStatus(std::string status);

  uint64_t getTimestampNanosec();
  uint64_t getTimestampMillisec();
  uint64_t getRosTimeNanosec(const std_msgs::Header& header); 

  geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& input_pose, const std::string& target_frame);
  geometry_msgs::Quaternion yawToQuaternion(double yaw);
  double quaternionToYaw(const geometry_msgs::Quaternion& quaternion);


  std::string getRemoteFrame(std::string aid_frame);
  uint8_t getAiosContainerType(uint8_t psaCntType);
  uint8_t getAiosJobType(uint8_t psaJobType);
  std::string getAiosDesinationtId(uint32_t psaDestinationId); 

  inline double getPingFrequency() { return ping_frequency_; }
  inline bool getLinkupStatus() { bool status = linkup_status_.load(); return status; }
  inline bool getPingAckStatus() { return ping_ack_status_; }
  inline uint64_t getLastPingMsgTimestamp() { return last_ping_msg_timestamp_; }
  inline uint64_t getLastConsoleAckSentTimestamp() { return  last_console_ack_sent_timestamp_; }
  inline double getPingTimeout() { return ping_timeout_; }
  inline uint64_t getPingRequestLatency() { return ping_request_latency_; }
  inline uint64_t getPingResponseLatency() { return ping_response_latency_; }
  inline bool getDebugStatus() { return debug_; }
  inline double getMainLoopRate(){return main_loop_rate_;}    
 
 
  inline bool isFirstPing() { bool status = is_first_ping_.load(); return status; }
  inline bool isBreaklinkEnabled() { return enable_breaklink_; }
  inline bool isSendPingRequest(){return send_ping_request_;}
  void autoResume();
 
   //Thread functions
  void sendEgoStateMsgToConsole();
  void sendTFMsgToConsole();

  

  ~RemoteHandler();
  
  uint64_t vehicle_ping_req_timestamp_;
  std::atomic<bool> run_thread_tf_;
  std::atomic<bool> run_thread_ego_;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  nlohmann::json json_aios_mqtt_config_;

  // Parameters
  std::string client_id_;
  std::string apm_id_ = "";
  std::string broker_address_;
  std::string to_client_topic_;
  std::string from_client_topic_;
  std::string vehicle_control_mode_topic_;
  std::string tf_topic_;
  std::string predicted_footprint_topic_;
  std::string route_plan_topic_;
  //PCD
  std::string vertical_points_topic_;
  std::string on_road_pc_topic_;
  std::string curb_pc_topic_;
  std::string most_constrained_points_topic_;
  std::string road_intensity_detection_topic_;
  //MARKERS
  std::string most_constrained_object_topic_; 
  std::string objects_topic_;
  std::string crosswalk_vis_topic_;
  std::string traffic_jam_lanes_vis_topic_;
  std::string road_global_vis_topic_;
  std::string objects_of_interest_topic_;
  std::string hatch_cover_detection_box_topic_;
  std::string rtg_detection_box_topic_;
 
    
  std::string ehmi_info_topic_;
  std::string instruction_source_;
  std::string mqtt_sub_topic_namespace_;
  std::string mqtt_pub_topic_namespace_;
  std::string to_aios_instructions_topic_;
  std::string from_aios_preprocess_topic_;
  std::string to_aios_ego_state_topic_;
  std::string to_aios_vehicle_control_mode_topic_;
  std::string to_aios_tl_state_topic_;
  std::string to_aios_remote_ops_init_topic_;
  std::string to_aios_linkup_status_topic_;
  std::string to_aios_job_info_topic_;
  std::string remote_ops_request_topic_;
  
  std::vector<nlohmann::json> tf_frames_;
  double tf_publish_rate_, ego_state_publish_rate_;

  std::string  aid_map_frame_;
  std::string  aid_odom_frame_;
  std::string  aid_base_link_frame_;
  std::string  remote_map_frame_;
  std::string  remote_odom_frame_;
  std::string  remote_base_link_frame_;

  // Remote Console message files in JSON format
  std::vector<nlohmann::json> json_msg_files_;
  std::map<std::string, int> json_pub_topic_mapping_;
  std::map<std::string, int> json_sub_topic_mapping_;

  std::string localization_status_topic_;
  double timeout_sec_;

  bool broker_connected_ = false;
  bool mqtt_initialized_ = false;

  ros::Time remote_connected_time_;

  ros::Publisher pub_to_mqtt_;
  ros::Publisher pub_to_aios_instructions_;
  ros::Publisher pub_to_aios_info_;
  ros::Publisher pub_to_ego_state_;
  ros::Publisher pub_to_vehicle_control_mode_;
  ros::Publisher pub_to_tl_state_;
  ros::Publisher pub_to_remote_ops_init_;
  ros::Publisher pub_to_remote_ops_request_;
  ros::Publisher pub_to_linkup_status_;
  ros::Publisher pub_to_job_info_;



  ros::Subscriber sub_to_aios_preprocess_;
  ros::Subscriber sub_to_vehicle_control_mode_;
  ros::Subscriber sub_to_predicted_footprint_;
  ros::Subscriber sub_to_route_plan_;
  ros::Subscriber sub_to_vertical_points_;
  ros::Subscriber sub_to_on_road_pc_;
  ros::Subscriber sub_to_curb_pc_;
  ros::Subscriber sub_to_most_constrained_points_;
  ros::Subscriber sub_to_road_intensity_detection_;
  ros::Subscriber sub_to_ehmi_info_;
  ros::Subscriber sub_to_mqtt_;
  ros::Subscriber sub_to_localization_status_;
  
  
  ros::Subscriber sub_to_most_constrained_object_;
  ros::Subscriber sub_to_objects_topic_;
  ros::Subscriber sub_to_crosswalk_vis_topic_;
  ros::Subscriber sub_to_traffic_jam_lanes_vis_topic_;
  ros::Subscriber sub_to_road_global_vis_topic_;
  ros::Subscriber sub_to_objects_of_interest_topic_;
  ros::Subscriber sub_to_hatch_cover_detection_box_topic_;
  ros::Subscriber sub_to_rtg_detection_box_topic_;
  
  
  ros::Subscriber sub_to_remote_ops_request_;

  ros::Timer error_reset_timer_;
  ros::Timer remote_ops_request_timer_;

  bool apm_preprocess_initialized_ = false;
 
  std::atomic<bool> linkup_status_;
  std::string linkup_vehicleID_ = "";
  std::string linkup_consoleID_ = "";

  uint8_t send_ping_seq_ = 0;
  uint8_t last_sent_ping_seq_ = 0;
  double ping_frequency_, ping_timeout_;
  bool ping_ack_status_ = false;
  std::atomic<bool> is_first_ping_;
  uint64_t last_ping_msg_timestamp_;
  uint64_t last_console_ack_sent_timestamp_ = 0;
  uint64_t ping_response_latency_ = 0;
  uint64_t ping_request_latency_ = 0;
  
  bool send_ping_request_;
  bool debug_ = false;
  bool enable_breaklink_;
  double main_loop_rate_;

  std::vector<aios_apm_msgs::AiosPreprocess> vec_send_ego_state_;

  float localization_status_;
  int  prevNudgeInstanceId_ = -1;
  std::map<uint8_t, bool>  nudge_joystick_release_; 
  std::map<uint8_t, bool>  nudge_joystick_push_; 

  std::mutex apm_preprocess_mtx_;
  std::string prev_avcs_destination_from_ui_ = "";
  std::string prev_ehmi_info_ = "";
  int prev_avcs_job_status_ = -1;
  int prev_tl_status_ = 99;

  bool error_reset_request_ = false;

  enum  EmergencyBrakeState {
        INITIAL = -1,
        OFF = 0,
        ON = 1,
        WAITING = 2
    };

  tf2_ros::Buffer tf_buffer_;
  std::atomic<EmergencyBrakeState> emergency_brake_on_; 
  std::atomic<bool>  isPause_;
  std::atomic<float> velocity_; 

  //alerts
  std::map<int, nlohmann::json> alerts_map_;
  std::map<int, nlohmann::json> metrics_map_;
  std::map<int, nlohmann::json> alert_metric_map_;
  
  std::vector<int> vec_alerts_id_;
  std::vector<std::string> vec_alerts_desc_;
  std::vector<uint8_t> restricted_actions_;

  nlohmann::json job_info_msg_;
  bool dest_first_link_up_ = false;
  bool job_status_first_link_up_ = false;
  
 
};

void ROS_INFO_STREAM_COLOUR(const std::string& colour, auto text) {
  std::unordered_map<std::string, std::string> colourMap{
      {"black", "0;30"}, {"red", "0;31"},     {"green", "0;32"}, {"yellow", "0;33"},
      {"blue", "0;34"},  {"magenta", "0;35"}, {"cyan", "0;36"},  {"white", "0;37"}};

  auto it = colourMap.find(colour);
  if (it != colourMap.end()) {
    ROS_INFO_STREAM("\033[" << it->second << "m" << text << "\033[0m");
  } else {
    ROS_INFO_STREAM(text);
  }
}

#endif