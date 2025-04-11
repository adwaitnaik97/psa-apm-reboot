// converted by Mahmoud Alsayed
// ROS standard msg Headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/console.h>
#include <nlohmann/json.hpp>
#include <string>
#include <std_msgs/Int8.h>
#include <ctime>
#include <chrono>
#include <cstdlib>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <regex>
#include <map>
#include <vector>
#include <algorithm>
#include <openssl/sha.h>
#include <aios_apm_msgs/V2iClientTokenStatus.h>
#include <aios_apm_msgs/AiosPreprocess.h>
#include <aios_apm_msgs/OntologyInfo.h>
#include <aios_apm_msgs/HttpsClientHeader.h>
#include <aios_apm_msgs/HttpsClient.h>
#include <fstream>
#include <iostream>

#include <aios_apm_msgs/V2I.h>
#include <aios_apm_msgs/V2I_intersection.h>
#include <aios_apm_msgs/V2I_state.h>
#include <aios_apm_msgs/V2iClientStatus.h>
#include <aios_apm_msgs/V2iIntersection.h>
#include <aios_apm_msgs/V2iStates.h>
#include <aios_apm_msgs/V2iTiming.h>
#include <aipe_msgs/Classification.h>

class V2iHandler
{
public:
    bool param_access_token_req;
    int  param_rate;
    bool _isAccessTokenValid = false;
    // std::ofstream logfile;

public:
    V2iHandler(ros::NodeHandle nh, ros::NodeHandle private_nh){
    // ros::Time timestamp = ros::Time::now();
    // Get the current time
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    // // Format the time as a string
    // std::tm* local_time = std::localtime(&now_time);
    // std::stringstream time_stream;
    // time_stream << std::put_time(local_time, "%Y-%m-%d_%H-%M-%S");
    //   logfile.open("ros_traffic_light_log_" + time_stream.str() + ".csv", std::ios::out | std::ios::app);
    //   if (!logfile.is_open()) {
    //       ROS_ERROR("Failed to open logfile.");
    //   }

    //   // Write header
    // logfile << "Time, aipe_traffic_light, v2i_handler_light, Ontology_junction, Group_ID, Event_state\n";


      private_nh.param<int>("rate", param_rate, 10);
      private_nh.param<bool>("use_service", param_use_service, false);
      private_nh.param<bool>("use_topic_request", param_use_topic_request, true);

      private_nh.param<bool>("access_token_req", param_access_token_req, true);
      private_nh.param<int>("token_expiry_seconds", param_token_expiry_seconds, 1200);
      private_nh.param<int>("request_timeout", param_request_timeout, 2);
      private_nh.param<std::string>("access_key", param_access_key, "49975406-f96a-4966-852a-1e8adf16cdd2");
      private_nh.param<std::string>("secret_key", param_secret_key, "e8b9a767-1e94-4c66-815b-e785863e1f27");

      private_nh.param<std::string>("access_token_url", param_access_token_url, "https://10.66.144.6/apim-token-service/v2.0/token/get");
      // private_nh.param<std::string>("access_token_url", param_access_token_url, "https://typedwebhook.tools/webhook/7cf8b2f8-aed0-4f14-beb3-fe8bb2f91cab");

      private_nh.param<std::string>("orgId", param_orgId, "o16263385430861578");

      https_request_pub = nh.advertise<aios_apm_msgs::HttpsClient>("https/to_client", 5);
      v2i_info_pub = nh.advertise<aios_apm_msgs::V2I>("info", 1);
      aipe_signal_pub = nh.advertise<aipe_msgs::Classification>("tl_signal", 5);

      // spesify the service name and the server type
      https_client_srv = nh.serviceClient<aios_apm_msgs::HttpsClientHeader>("https_request");
      

      // client_tl_id_sub = private_nh.subscribe<std_msgs::String>("/aipe/junction_id", 1, &V2iHandler::client_tl_id_callback, this, ros::TransportHints().tcpNoDelay(true));
      preprocess_sub = private_nh.subscribe<aios_apm_msgs::AiosPreprocess>("/aios/preprocess/info", 1, &V2iHandler::preprocess_callback, this, ros::TransportHints().tcpNoDelay(true));
      next_tl_junction_sub = private_nh.subscribe<aios_apm_msgs::OntologyInfo>("/aios/ontology/info", 1, &V2iHandler::ontology_callback, this, ros::TransportHints().tcpNoDelay(true));
      // v2i_handler_debug_sub = private_nh.subscribe<aios_apm_msgs::V2I>("/aios/v2i_handler/info", 1, &V2iHandler::v2i_handler_debug_callback, this, ros::TransportHints().tcpNoDelay(true));
      // aipe_traffic_v2i_sub = private_nh.subscribe<aipe_msgs::Classification>("/aipe/tl_signal/v2i", 1, &V2iHandler::aipe_v2i_callback, this, ros::TransportHints().tcpNoDelay(true));
      // aipe_traffic_light_sub = private_nh.subscribe<aipe_msgs::Classification>("/aipe/traffic_light/signal", 1, &V2iHandler::aipe_trafficLight_callback, this, ros::TransportHints().tcpNoDelay(true));
      // v2i_handler_sub = private_nh.subscribe<aipe_msgs::Classification>("/aios/v2i_handler/tl_signal/v2i_2", 1, &V2iHandler::v2i_handler_callback, this, ros::TransportHints().tcpNoDelay(true));
      https_response_sub = nh.subscribe<std_msgs::String>("https/from_client", 10, &V2iHandler::https_response_callback, this, ros::TransportHints().tcpNoDelay(true));
      
      V2iClientTokenStatus_msg= aios_apm_msgs::V2iClientTokenStatus();

      access_token_header_["Content-Type"] = "application/json";
      access_token_payload_["appKey"] = param_access_key;
      // _tl_id_str = "ppt12";

      // intersection_header;
      // intersection_payload;

    };

    ~V2iHandler() {
        // Ensure logfile is closed when the object is destroyed
 
      //  if (logfile.is_open()) {
      //       logfile.close();
      //   }

    }
    
    // Log function
    // void logValues() {
        
    //     auto current_time = std::chrono::system_clock::now();
    //     std::time_t converted_time = std::chrono::system_clock::to_time_t(current_time);

    //     // Format the time as a string
    //     std::tm* local_time_ = std::localtime(&converted_time);
    //     std::stringstream time_stream_;
    //     time_stream_ << std::put_time(local_time_, "%Y-%m-%d_%H-%M-%S");
    //     if(aipe_traffic_light != tf_signal_msg.class_name)
    //     {
    //       logfile << time_stream_.str() << "," << aipe_traffic_light << "," << tf_signal_msg.class_name << "," << Ontology_junction << "," << group_ID << "," << event_id << std::endl;
    //     }
    //     // logfile << timestamp << "," << aipe_traffic_light << "," << v2i_control_light << "," << v2i_handler_light << "," << aipe_junction << "," << Ontology_junction << std::endl;
    //     // ROS_INFO_STREAM("Logged at: " << timestamp << " | Topic 1: " << aipe_traffic_light 
    //     //                 << " | Topic 2: " << v2i_control_light 
    //     //                 << " | Topic 3: " << v2i_handler_light);
    // }

    // void aipe_v2i_callback(const aipe_msgs::Classification::ConstPtr& msg){
    //   v2i_control_light = msg->class_name;
    //   // logValues();
    // }

    // void aipe_trafficLight_callback(const aipe_msgs::Classification::ConstPtr& msg){
    //   aipe_traffic_light = msg->class_name;
    //   // std::cout << "======================================================================" << std::endl;
    //   // std::cout << "junction_id: " << Ontology_junction << std::endl;
    //   // std::cout << "signal group: " << group_ID << std::endl;
    //   // std::cout << "state: " << state_ << std::endl;
    //   // logValues();
    // }

    // void v2i_handler_callback(const aipe_msgs::Classification::ConstPtr& msg){
    //   v2i_handler_light = msg->class_name;
    //   logValues();
    // }

    void https_response_callback(const std_msgs::String::ConstPtr& msg){
      // ROS_INFO_STREAM(msg->data);
      if(v2i_client_status_msg.token_status.token_validation_pending)
      {
        // std::cout << "--------------2------------------" << std::endl;
        parseAccessToken(msg->data);
      }
      else if(v2i_client_status_msg.intersection_parsing)
      {
        // std::cout << "--------------1------------------" << std::endl;
        parseIntersection(msg->data);
      }
    }

    // void client_tl_id_callback(const std_msgs::String::ConstPtr& msg){
    //   aipe_junction = msg->data;
    // }

    // Function to check the if indicator is signaling
    void preprocess_callback(const aios_apm_msgs::AiosPreprocess::ConstPtr& msg){
      apm_id = msg->platform_info.platform_id;

      light_indicator = msg->airs_info.indicator_light_status;
    }
    // Check which group the APM belongs to.
    // void v2i_handler_debug_callback(const aios_apm_msgs::V2I::ConstPtr& msg){
    //   if (msg->intersections.size() > 0) {
    //     for (auto state : msg->intersections[0].states) {
    //       // if the signal group is same as ID, print out the event state
    //       if (state.signal_group == group_ID){
    //         event_state(state.event_state);
    //         std::cout << "================: " << state.event_state << std::endl;
    //       }
    //     }
    //   }

    //   // if (v2i_client_status_msg.intersections.size() > 0) {
    //   //   for (auto state : v2i_client_status_msg.intersections[0].states) {
    //   //     // if the signal group is same as ID, print out the event state
    //   //     if (state.signal_group == group_ID){
    //   //       event_state(state.event_state);
    //   //     }
    //   //   }
    //   // }


      
      
    // }

    // Check the event state and print out whihc light colour it is 
    void event_state(int event_state_val){
      state_ = event_state_val;
      if (next_tl_junction == ""){
        tf_signal_msg.class_name = "";
        tf_signal_msg.class_index = -1;
      } 
      else if (next_tl_junction != ""){
        // if (event_state_val == 3){
        //   tf_signal_msg.class_name = "red";
        //   tf_signal_msg.class_index = 0;
        //   tf_signal_msg.confidence = 1;
        // }
        if (event_state_val == 6){
          tf_signal_msg.class_name = "green";
          tf_signal_msg.class_index = 2;
          tf_signal_msg.confidence = 1;
          if (light_indicator == 1){
            tf_signal_msg.class_name = "right_arrow";
            tf_signal_msg.class_index = 3;
            tf_signal_msg.confidence = 1;
          }
        }
        else{
          tf_signal_msg.class_name = "red";
          tf_signal_msg.class_index = 0;
          tf_signal_msg.confidence = 1;
        }
      }
    }

      // Check which group it belongs to 
      void check_group() {
        std::transform(ppt_junction.begin(), ppt_junction.end(), ppt_junction.begin(), ::toupper);
        if (apm_direction_frame == "FRAME_N" || apm_direction_frame == "FRAME_S") {
          if (ppt_junction == "PPT10" || ppt_junction == "PPT12" || ppt_junction == "PPT13" || ppt_junction == "PPT14" || ppt_junction == "PPT15"){
            if (light_indicator == 1) {
              group_ID = 2;
            } 
            else if (light_indicator == 0) {
              group_ID = 1;
            } else {
              // ROS_INFO_STREAM("Does not belong to a group");
              tf_signal_msg.class_name = "";
              tf_signal_msg.class_index = -1;
            }
          }
          else if(ppt_junction == "PPT16"){
            if (light_indicator == 0) {
              // ROS_INFO_STREAM("Does not belong to a group");
            } 
            else if (light_indicator == 1) {
              group_ID = 3;
              if (lane_type_msg == 21){
                group_ID = 4;
              }
            } 
            else if (light_indicator == -1) {
              // ROS_INFO_STREAM("Does not belong to a group");
              tf_signal_msg.class_name = "";
              tf_signal_msg.class_index = -1;
            } 
          }

          else if(ppt_junction == "PPT11"){
            if(light_indicator == 1){
              group_ID = 3;
            }
            else if(light_indicator == 0){
              if(apm_direction_frame == "FRAME_N"){
                group_ID = 1;
              }
              else{
                group_ID = 2;
              }
            }

          }
          
          else{
            if (light_indicator == 1) {
                group_ID = 1;
            } 
            else if (light_indicator == 0) {
                group_ID = 2;
                // ROS_INFO_STREAM("I am here 0");
            } 
            else {
                // ROS_INFO_STREAM("Does not belong to a group");
                tf_signal_msg.class_name = "";
                tf_signal_msg.class_index = -1;
            }
          }
        } 
        else if (apm_direction_frame == "FRAME_W" || apm_direction_frame == "FRAME_E") { 
          if (ppt_junction == "PPT16"){
            if (light_indicator == 1) {
              group_ID = 5;
            } 

            else if (light_indicator == 0) {
              if (apm_direction_frame == "FRAME_E"){
                group_ID = 1;
              }
              else{
                group_ID = 2;
              }
              
            } 
            else if (light_indicator == 2){
              group_ID = 2;
            }
            else if (light_indicator == -1) {
              group_ID = 2;
              tf_signal_msg.class_name = "";
              tf_signal_msg.class_index = -1;
            } 
          }

          else if (ppt_junction == "PPT10" || ppt_junction == "PPT11" || ppt_junction == "PPT12" || ppt_junction == "PPT13" || ppt_junction == "PPT14" || ppt_junction == "PPT15"){
            if (light_indicator == 1) {
              group_ID = 4;
            } 
            else if (light_indicator == 0) {
              group_ID = 3;
            } 
            else if (light_indicator == -1) {
              // ROS_INFO_STREAM("Does not belong to a group");
              tf_signal_msg.class_name = "";
              tf_signal_msg.class_index = -1;
            } 
          }
          else if(ppt_junction == "PPT09"){
            if(light_indicator == 1){
              group_ID = 3;
            }
            else {
              // ROS_INFO_STREAM("Does not belong to a group");
              tf_signal_msg.class_name = "";
              tf_signal_msg.class_index = -1;
            }
          }
          
        }
        else{
          tf_signal_msg.class_name = "";
          tf_signal_msg.class_index = -1;
        }

        v2i_client_status_msg.signal_group = group_ID;
      }


    // Check the next TL junction and convert it to its name 
    void ontology_callback(const aios_apm_msgs::OntologyInfo::ConstPtr& msg){
      next_tl_junction = msg->junctions.next_tl_junction;
      apm_direction_frame = msg->current_lane_info.direction_frame;
      lane_type_msg = msg->current_lane_info.lane_type;
      _tl_id_str = msg->junctions.next_tl_junction_alias;
      std::transform(_tl_id_str.begin(), _tl_id_str.end(), _tl_id_str.begin(), ::tolower);
      Ontology_junction = msg->junctions.next_tl_junction_alias;
      v2i_client_status_msg.requested_intersection = _tl_id_str;

      if(msg->junctions.next_tl_junction_alias == "")
        return;
      ppt_junction = msg->junctions.next_tl_junction_alias;

      // if(msg->junctions.next_tl_junction == "")
      //   return;
        
      // for (const auto &[key, value] : my_map){
      //   if (key == next_tl_junction){
      //     ppt_junction = value;
      //     v2i_client_status_msg.requested_intersection = value;
      //   }
      // }
    }

    void PublishInfo(){
        v2i_info_pub.publish(v2i_client_status_msg);
        if (v2i_client_status_msg.intersections.size() > 0) {
          for (auto state : v2i_client_status_msg.intersections[0].states) {
            // if the signal group is same as ID, print out the event state
            if (state.signal_group == group_ID){
              event_state(state.event_state);
              event_id = state.event_state;
              // std::cout << "================: " << state.event_state << std::endl;
            }
          }
        }
        tf_signal_msg.header.stamp = ros::Time::now();
        aipe_signal_pub.publish(tf_signal_msg);

    }

    // void tf_signalPublish(){
    //     aipe_signal_pub.publish(tf_signal_msg);
    // }

    void parseAccessToken(std::string response_data){
      if(!response_data.empty()){
        // Convert the JSON string to an nlohmann::json object
        try{
          nlohmann::json token_raw_msg_json = nlohmann::json::parse(response_data);
          _access_token = token_raw_msg_json["data"]["accessToken"];
          // ROS_INFO_STREAM(_access_token);
          if(_access_token != "")
          {
            V2iClientTokenStatus_msg.atoken_status= true;
            _isAccessTokenValid = V2iClientTokenStatus_msg.atoken_status;
            V2iClientTokenStatus_msg.atoken = _access_token;
            V2iClientTokenStatus_msg.atoken_expiry_at_reg = token_raw_msg_json["data"]["expire"];
            _latest_token_time = ros::Time::now();
             v2i_client_status_msg.token_status.token_validation_pending = false;
          }

        }
        catch(const std::exception& e){
          ROS_ERROR_STREAM(e.what());
        }
      }
    }

    void parseIntersection(std::string response_data){
      if(!response_data.empty()){
        // Convert the JSON string to an nlohmann::json object
        try
        {
            nlohmann::json intersection_raw_json = nlohmann::json::parse(response_data);
            if(intersection_raw_json["code"] != 200 ){
                return;
            }
            else{
                  v2i_client_status_msg.intersection_parsing = false;
                  v2i_client_status_msg.requested_intersection = _tl_id_str;
                  v2i_client_status_msg.header.stamp = ros::Time::now();
                  v2i_client_status_msg.timestamp = intersection_raw_json["data"]["timestamp"];

                  v2i_client_status_msg.intersections.clear();


                  for (auto intersection : intersection_raw_json["data"]["intersections"]) {                
                      aios_apm_msgs::V2I_intersection intersection_msg;
                      intersection_msg.intersection_id = intersection["id"];
                      intersection_msg.status = intersection["status"];
                      intersection_msg.timestamp = intersection["timestamp"];

                      for (auto state : intersection["states"]) {      
                          aios_apm_msgs::V2I_state states_msg;
                          states_msg.signal_group = state["signal_group"];
                          states_msg.event_state = state["event_state"];
                          states_msg.min_end_time = state["timing"]["min_end_time"];
                          states_msg.max_end_time = state["timing"]["max_end_time"];
                          states_msg.likely_time = state["timing"]["likely_time"];
                          intersection_msg.states.push_back(states_msg);
                      }
                      v2i_client_status_msg.intersections.push_back(intersection_msg);
                  }
             }
        }
        catch(const std::exception& e){
          ROS_ERROR_STREAM(e.what());
        }
      }
    }

    void checkAccessTokenValid()
    {
      if(!_isAccessTokenValid)
        return;

      // check access token expiry then flag access token invalid
      float token_expiry = (ros::Time::now() - _latest_token_time).toSec();
      if(token_expiry > V2iClientTokenStatus_msg.atoken_expiry_at_reg || token_expiry > param_token_expiry_seconds)
      {
        V2iClientTokenStatus_msg.atoken_status = false;
        _isAccessTokenValid = V2iClientTokenStatus_msg.atoken_status;
        V2iClientTokenStatus_msg.atoken = "";
        ROS_ERROR_STREAM("Token invalid");
      }
      else
      {
        V2iClientTokenStatus_msg.atoken_expiry_remain = V2iClientTokenStatus_msg.atoken_expiry_at_reg - token_expiry;
      }

      v2i_client_status_msg.token_status = V2iClientTokenStatus_msg;
    }

    void accessTokenRequest(){
        // Get the current ROS time
        ros::Time now = ros::Time::now();
        // Convert the time to seconds and then to milliseconds
        int64_t time_milliseconds = static_cast<int64_t>(now.toSec() * 1000);
        // Convert the result to a string
        std::string timestamp_now = std::to_string(int64_t(round(time_milliseconds)));
        std::string encrypt_token = encrypt_sha256(param_access_key + timestamp_now + param_secret_key);

        access_token_payload_["encryption"] = encrypt_token;
        access_token_payload_["timestamp"] = timestamp_now;
        


        if(param_use_service)
        {
            aios_apm_msgs::HttpsClientHeader srv = aios_apm_msgs::HttpsClientHeader();
            srv.request.type = "POST";
            srv.request.url = param_access_token_url;
            srv.request.headers = access_token_header_.dump();
            srv.request.payload = access_token_payload_.dump();
            srv.request.timeout = param_request_timeout;
            if (https_client_srv.call(srv))
            {
                parseAccessToken(srv.response.server_response);
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }
        } 
           

        if(param_use_topic_request)
        {
          aios_apm_msgs::HttpsClient request_msg;
          request_msg.is_valid = true;
          request_msg.url = param_access_token_url;
          request_msg.headers = access_token_header_.dump();
          request_msg.payload = access_token_payload_.dump();
          request_msg.timeout = param_request_timeout;
          request_msg.type = "POST";

          https_request_pub.publish(request_msg);
          // std::cout << "-------------3-------------" << std::endl;
          v2i_client_status_msg.token_status.token_validation_pending = true;
        }
    }

    void dataReceive(){
        if(apm_id.empty()){
           ROS_ERROR_STREAM_THROTTLE(5,"apm id to be set");
           return;
        };

        if(_tl_id_str.empty()){
           ROS_ERROR_STREAM_THROTTLE(5,"intersection id to be set");
           return;
        };


        ros::Time now = ros::Time::now();
        // Convert the time to seconds and then to milliseconds
        int64_t time_milliseconds = static_cast<int64_t>(now.toSec() * 1000);
        // Convert the result to a string
        std::string timestamp_now;
        timestamp_now = std::to_string(int64_t(round(time_milliseconds)));
        // add headers key, value pair in json format , and compute the n2h256 hash (Algorithm 256-bit)
        intersection_header["apim-accesstoken"] = _access_token;
        intersection_header["apim-timestamp"] = timestamp_now;
        // apim-signature encryption order:
        // AccessToken + "client_id" + clientID + "id" + IntersectionID + "orgId" + orgID + time + secretKey
        std::string encrypted_apim_signature = encrypt_sha256(_access_token + "client_idapm" + apm_id + "id" + _tl_id_str + "orgId" + 
                                                            param_orgId + timestamp_now + param_secret_key );

        intersection_header["apim-signature"] = encrypted_apim_signature;  
        intersection_url = intersection_base_url + apm_id + "&id=" + _tl_id_str + "&orgId=" + param_orgId;
        // intersection_url = "https://typedwebhook.tools/webhook/7cf8b2f8-aed0-4f14-beb3-fe8bb2f91cab";
        // ROS_ERROR_STREAM(intersection_url);

        // intersection_payload.clear();

        if(param_use_service)
        {
            aios_apm_msgs::HttpsClientHeader intersection_srv = aios_apm_msgs::HttpsClientHeader();
            intersection_srv.request.type = "GET";
            intersection_srv.request.url = intersection_url;
            intersection_srv.request.headers = intersection_header.dump();
            intersection_srv.request.payload = intersection_payload.dump();
            intersection_srv.request.timeout = param_request_timeout;
            if (https_client_srv.call(intersection_srv))
            {
                parseIntersection(intersection_srv.response.server_response);
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }
        } 
           

        if(param_use_topic_request)
        {
          aios_apm_msgs::HttpsClient request_msg;
          request_msg.is_valid = true;
          request_msg.url = intersection_url;
          request_msg.headers = intersection_header.dump();
          request_msg.payload = intersection_payload.dump();
          request_msg.timeout = param_request_timeout;
          request_msg.type = "GET";

          https_request_pub.publish(request_msg);
          // std::cout << "-------------4-------------" << std::endl;
          v2i_client_status_msg.intersection_parsing = true;
        }
    }

      
    std::string encrypt_sha256(const std::string str) {
      unsigned char hash[SHA256_DIGEST_LENGTH];
      SHA256((const unsigned char*)str.c_str(), str.length(), hash);

      std::stringstream ss;
      for (int i = 0; i < SHA256_DIGEST_LENGTH; ++i) {
          ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
      } 

      return ss.str();
    }

   


  private:
  
  // std::map<std::string, std::string> my_map = {
  //       { "TLJ-PPA7-PPA8", "PPT09" },
  //       { "TLJ-TAB-PPA8", "PPT10" },
  //       { "TLJ-TBC-PPA8", "PPT11" },
  //       { "TLJ-TCD-PPA8", "PPT12" },
  //       { "TLJ-TDE-PPA8", "PPT13" },
  //       { "TLJ-TEF-PPA8", "PPT14" },
  //       { "TLJ-TFG-PPA8", "PPT15" },
  //       { "TLJ-TMP-PPD8", "PPT16" }
  //   };

  std::string _tl_id_str = "ppt12";

  std::string param_access_token_url;
  std::string param_access_key;
  std::string param_secret_key;
  std::string param_orgId;
  std::string apm_direction_frame;
  int lane_type_msg;
  int light_indicator;
  int group_ID;
  int state_;
  int event_id;
  std::string turning_Left;
  std::string going_straight;
  std::string turning_right;

  std::string aipe_traffic_light;
  std::string v2i_control_light;
  std::string v2i_handler_light;
  std::string aipe_junction;
  std::string Ontology_junction;
  

  nlohmann::json access_token_payload_;
  nlohmann::json access_token_header_;

  nlohmann::json intersection_header;
  nlohmann::json intersection_payload;
  std::string intersection_base_url = "https://10.66.144.6/portal-ppe1/v1.0/trafficlight/traffic-lights/apm/v1/intersection?client_id=apm";
  std::string intersection_url;


  std::string _access_token = "";
  ros::Time _latest_token_time = ros::Time::now();

  int param_server_recieve_retry;
  int param_token_expiry_seconds;
  int param_request_timeout;
  std::string apm_id;
  std::string next_tl_junction;
  std::string ppt_junction;

  bool param_use_service;
  bool param_use_topic_request;



  ros::Publisher v2i_info_pub;
  ros::Publisher https_request_pub;
  ros::Publisher aipe_signal_pub;

  ros::Subscriber client_tl_id_sub;
  ros::Subscriber preprocess_sub;
  ros::Subscriber https_response_sub;
  ros::Subscriber next_tl_junction_sub;
  ros::Subscriber apm_direction_frame_sub;
  ros::Subscriber indictor_light_status_sub;
  ros::Subscriber v2i_handler_debug_sub;
  ros::Subscriber aipe_traffic_v2i_sub;
  ros::Subscriber aipe_traffic_light_sub;
  ros::Subscriber v2i_handler_sub;


  ros::ServiceClient https_client_srv ;

  aios_apm_msgs::V2I v2i_client_status_msg;
  aipe_msgs::Classification tf_signal_msg;
  aios_apm_msgs::V2iClientTokenStatus V2iClientTokenStatus_msg;

};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "v2i_handler_node"); // Initialize the ROS node
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // Creating a private namespace with "~"

    V2iHandler V2iHandler_object(nh, private_nh);
    ros::Rate r(V2iHandler_object.param_rate);

    

    while (ros::ok())
    {
      if(V2iHandler_object.param_access_token_req == true && V2iHandler_object._isAccessTokenValid == false){
        V2iHandler_object.accessTokenRequest();
      }
      else{
        V2iHandler_object.dataReceive();

      }
      if(V2iHandler_object.param_access_token_req == true){
        V2iHandler_object.checkAccessTokenValid();        
      }

      V2iHandler_object.check_group();
      V2iHandler_object.PublishInfo();
      // V2iHandler_object.logValues();
      // V2iHandler_object.tf_signalPublish();
      // V2iHandler_object.v2i_handler_debug_callback();
     
    ros::spinOnce();
    r.sleep();
  }


  return 0;
  }        




