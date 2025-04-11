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
 * author = 'Zeynep Ustun'
 * email  = 'zeynep@aidrivers.ai'
 *
 *******************************************************/

#include <MQTTAsync.h>
#include <remote_ops/remote_ops.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include "mqtt/async_client.h"

std::vector<nlohmann::json> vec_send_console_ping_ack;
std::vector<nlohmann::json> vec_ego_state;
std::shared_ptr<RemoteOps> remoteops_ptr = nullptr;

std::atomic<bool> run_thread_ego_state(true);
std::atomic<bool> run_thread_ping_ack(true);

std::mutex mtx_to_client_vec;
std::mutex mtx_ego_state_vec;
std::mutex mtx_console_ping_ack;
std::mutex mtx_new_msg_flag_;

std::mutex mtx_from_client_vec;
std::vector<std::string> from_client_vec;

const int N_RETRY_ATTEMPTS = 5;  // Counter for the number of connection retries
const int COMMAND_QOS = 2;
const int VIEW_QOS = 0;

class action_listener : public virtual mqtt::iaction_listener {
  std::string name_;

  void on_failure(const mqtt::token& tok) override {
    std::cout << name_ << " failure";
    if (tok.get_message_id() != 0)
      std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    std::cout << std::endl;
  }

  void on_success(const mqtt::token& tok) override {
    std::cout << name_ << " success";
    if (tok.get_message_id() != 0)
      std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    auto top = tok.get_topics();
    if (top && !top->empty()) std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
    std::cout << std::endl;
  }

 public:
  action_listener(const std::string& name) : name_(name) {}
};

class callback : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener

{
 public:
  callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
      : nretry_(1), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {
    pub_incoming_payload_ = nh_.advertise<std_msgs::String>("/remoteops/incoming_payload", 1);
  }

  void reconnect() {
    std::cout << "------------MQTT Client trying to reconect------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    try {
      cli_.connect(connOpts_, nullptr, *this);
    } catch (const mqtt::exception& exc) {
      std::cerr << "Error: " << exc.what() << std::endl;
    }
  }

  // Re-connection failure
  void on_failure(const mqtt::token& tok) override {
    std::cout << "MQTT Client-Connection attempt failed, retrying in 1s" << std::endl;
    broker_connected_ = false;
    sleep(2);
    reconnect();
  }

  // (Re)connection success
  void on_success(const mqtt::token& tok) override {}

  // (Re)connection success
  void connected(const std::string& cause) override {
    std::cout << "\nMQTT Client - Connection success" << std::endl;

    std::string linkup_topic_name = mqtt_sub_topic_ns_ + "linkup";
    cli_.subscribe(linkup_topic_name, COMMAND_QOS, nullptr, subListener_);

    subscribed_topics_.clear();
    subscribed_topics_.push_back(linkup_topic_name);
    broker_connected_ = true;
  }

  // Callback for when the connection is lost.
  // This will initiate the attempt to manually reconnect.
  void connection_lost(const std::string& cause) override {
    std::cout << "\n[MQTT Client] Connection lost" << std::endl;
    if (!cause.empty()) std::cout << "\tcause: " << cause << std::endl;

    if (nretry_ < N_RETRY_ATTEMPTS) {
      std::cout << "[MQTT Client] Reconnecting [" << nretry_ << "/" << N_RETRY_ATTEMPTS
                << " attempt]" << std::endl;
      reconnect();
      nretry_++;
    } else {
      std::cout << "[MQTT Client] - Retry attemtps..." << std::endl;
    }
  }

  // MQTT broker message arrives.
  void message_arrived(mqtt::const_message_ptr msg) override {
    try
    {
      uint64_t currentTimestamp = remoteops_ptr->getTimestamp();
      std::string str_payload = msg->to_string();
      std::string hex_payload = remoteops_ptr->getHexPayload(str_payload);

      std::string topic_name = msg->get_topic();
      std::string remote_topic = "remoteops";
     
      if (topic_name.find(remote_topic) == std::string::npos) //not found
      {
        return;
      }  
     

      // visualization topics
      if (std::find(remoteops_ptr->vec_visualization_topic_.begin(),
                    remoteops_ptr->vec_visualization_topic_.end(),
                    topic_name) != remoteops_ptr->vec_visualization_topic_.end()) {
        return;
      }

      std::string msg_type = topic_name;
      std::size_t found = topic_name.rfind('/');

      if (found != std::string::npos) {
        msg_type = topic_name.substr(found + 1);
      }
      uint8_t operation_type = remoteops_ptr->getOperationType(msg_type) ;
      

      nlohmann::json deserialized_msg;

      bool errorRaised = false;
      bool rejected = false;
      std::string reject_reason = "";
      bool isValidConsoleRequest = remoteops_ptr->deserializeMsgFromConsole(msg, deserialized_msg, errorRaised, rejected, reject_reason);

      if (errorRaised)
      {
        return;
      }
      
      if ((rejected) && (operation_type != 99)) 
      {
        sendOpsResponse(operation_type, currentTimestamp, reject_reason);
        return;
      } 

      if (!isValidConsoleRequest) {
        // Console Ping Ack message to Vehicle
        if (deserialized_msg["payload"].contains("requestLatency")) {
          // calculate console ack response latency
          uint64_t ack_ts = deserialized_msg["timestamp"];

          auto ack_time_diff = (currentTimestamp - ack_ts);
          std::chrono::nanoseconds ack_duration_ns(ack_time_diff);
          std::chrono::milliseconds ack_duration_ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(ack_duration_ns);
          uint64_t ack_response_latency_ms = ack_duration_ms.count();

          // nlohmann::json pub_msg;
          // pub_msg["sender"] = "console";
          // pub_msg["timestamp"] = ack_ts;
          // pub_msg["topic_name"] = deserialized_msg["topicName"];
          // pub_msg["payload"] = deserialized_msg["payload"];
          // pub_msg["response_latency_ms"] = ack_response_latency_ms;
          // remoteops_ptr->publishPayloadToROS(pub_incoming_payload_, pub_msg);

          std_msgs::String msg;
          deserialized_msg["response_latency_ms"] = ack_response_latency_ms;
          deserialized_msg["console_ack_sent_ts"] = ack_ts;

          msg.data = deserialized_msg.dump();

          pub_from_client_.publish(msg);
          if (remoteops_ptr->debug_) {
            nlohmann::json write_ping_msg_json;
            write_ping_msg_json["console_ack_sent_ts"] = ack_ts;
            write_ping_msg_json["vehicle_ack_recieve_ts"] = currentTimestamp;
            write_ping_msg_json["response_latency_ms"] = ack_response_latency_ms;
            write_ping_msg_json["request_latency_ms"] = deserialized_msg["payload"]["requestLatency"];
            write_ping_msg_json["seq"] = deserialized_msg["payload"]["seq"];

            std::string write_ping_msg = write_ping_msg_json.dump() + "\n";
            remoteops_ptr->writeMsgToFile("ping.txt", write_ping_msg, "incoming_payload", true);
          }

        } else if (deserialized_msg["topicName"] == "/remoteops/breaklink") {
          if (deserialized_msg["payload"].contains("breakLinkReason") &&
              remoteops_ptr->getVehicleBreakLinkRequest()) {
            std::string msg_out =
                "*****4-DONOT-MQTT msg-VehicleBreaklinkRequest:" + deserialized_msg.dump();
            remoteops_ptr->ROS_INFO_STREAM_COLOR("magenta", msg_out);
          } else if (deserialized_msg["payload"].contains("breakLinkReason") &&
                    !remoteops_ptr->getVehicleBreakLinkRequest()) {
            std::string msg_out =
                "*****6-DONOT-MQTT msg-ConsoleBreaklinkRequest::" + deserialized_msg.dump();
            remoteops_ptr->ROS_INFO_STREAM_COLOR("magenta", msg_out);
          }

          else if (!deserialized_msg["payload"].contains("breakLinkReason") &&
                  !remoteops_ptr->getVehicleBreakLinkRequest())  // vehicle ACK
          {
            std::string msg_out = "7-DONOT-MQTT msg-VEHICLE BreaklinkACK:" + deserialized_msg.dump();
            remoteops_ptr->ROS_INFO_STREAM_COLOR("magenta", msg_out);
          } else if (!deserialized_msg["payload"].contains("breakLinkReason") &&
                    remoteops_ptr->getVehicleBreakLinkRequest()) {
            std::string msg_out = "4-PUBLISH-MQTT msg-ConsoleBreaklinkACK:" + deserialized_msg.dump();
            remoteops_ptr->ROS_INFO_STREAM_COLOR("magenta", msg_out);

            nlohmann::json pub_msg;
            pub_msg["sender"] = "console";
            pub_msg["timestamp"] = deserialized_msg["timestamp"];
            pub_msg["topic_name"] = deserialized_msg["topicName"];
            pub_msg["payload"] = deserialized_msg["payload"];
            // pub_msg["hex_payload"] = hex_payload;

            std::string file_name = "breaklink_" + to_string(deserialized_msg["timestamp"]) + ".txt";
            remoteops_ptr->writeMsgToFile(file_name, str_payload, "incoming_payload");
            remoteops_ptr->writeMsgToFile(file_name, hex_payload, "incoming_payload/hex");
            // remoteops_ptr->publishPayloadToROS(pub_incoming_payload_, pub_msg);

            std_msgs::String msg;
            msg.data = deserialized_msg.dump();

            pub_from_client_.publish(msg);
          }
        }
        return;
      }
      
      // {
      //   std::lock_guard<std::mutex> lock(mtx_new_msg_flag_);
      //   new_msg_ = true;
      // }
      

      // Console Ping Request msg arrived
      if (msg_type == "ping") {
        remoteops_ptr->console_ping_arrive_ts_ = currentTimestamp;
        uint64_t messageTimestamp = deserialized_msg["timestamp"];
        auto time_diff = (currentTimestamp - messageTimestamp);
        std::chrono::nanoseconds duration_ns(time_diff);
        std::chrono::milliseconds duration_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(duration_ns);
        uint64_t request_latency_ms = duration_ms.count();

        remoteops_ptr->setVehicleAckPingRequestLatency(request_latency_ms);
        deserialized_msg["calculatedRequestLatency"] = request_latency_ms;

        nlohmann::json ping_ack;

        ping_ack["topicName"] = "/remoteops/ping";
        ping_ack["msgType"] = "ping";
        ping_ack["sender"] = deserialized_msg["payload"]["receiver"];
        ping_ack["receiver"] = deserialized_msg["payload"]["sender"];
        ping_ack["seq"] = deserialized_msg["payload"]["seq"];
        ping_ack["requestLatency"] = request_latency_ms;

        if (request_latency_ms > 120) {
          ROS_WARN_STREAM("Request Latency(ms) WARN:" << request_latency_ms);
        }

        { 
          std::lock_guard<std::mutex> lock(mtx_console_ping_ack);
          vec_send_console_ping_ack.push_back(ping_ack);
        }
        // if (remoteops_ptr->debug_) {
        //   // debug latency
        //   nlohmann::json write_ping_msg_json;
        //   write_ping_msg_json["seq"] = deserialized_msg["payload"]["seq"];
        //   write_ping_msg_json["console_ping_req_sent_ts"] = messageTimestamp;
        //   write_ping_msg_json["vehicle_ping_req_receive_ts"] = currentTimestamp;
        //   write_ping_msg_json["request_latency_ms"] = duration_ms.count();

        //   std::string write_ping_msg = write_ping_msg_json.dump() + "\n";
        //   // ROS_WARN_STREAM(write_ping_msg);
        //   remoteops_ptr->writeMsgToFile(msg_type + ".txt", write_ping_msg, "outgoing_payload", true);
        // }
      }
      
      {
      //  std::lock_guard<std::mutex> lock(mtx_new_msg_flag_); 
      //  payload_str_msg_ = deserialized_msg.dump();
          std::string from_client_msg = deserialized_msg.dump();
          std::lock_guard<std::mutex> lock(mtx_from_client_vec);
          from_client_vec.push_back(from_client_msg); 
      }
      
      
      
      if (msg_type != "ping") {
      
        //OPS_RESPONSE
        if ( operation_type != 99) 
        {   
          sendOpsResponse(operation_type,  currentTimestamp, ""); 
        }
        // publish message to ROS topic
        nlohmann::json pub_msg;
        pub_msg["sender"] = "console";
        pub_msg["timestamp"] = deserialized_msg["timestamp"];
        pub_msg["topic_name"] = deserialized_msg["topicName"];
        pub_msg["payload"] = deserialized_msg["payload"];

        std::string file_name = msg_type + "_" + to_string(deserialized_msg["timestamp"]) + ".txt";

        remoteops_ptr->writeMsgToFile(file_name, str_payload, "incoming_payload");
        //remoteops_ptr->writeMsgToFile(file_name, hex_payload, "incoming_payload/hex");

        remoteops_ptr->publishPayloadToROS(pub_incoming_payload_, pub_msg);
      }
    }
    catch (nlohmann::json::type_error& e) {
      std::cerr << "1-message_arrived: Exception[type_error]-" << e.what() << std::endl;
    }
    catch (const std::exception& e) {
      std::cerr << "2-message_arrived: Exception[std]-" << e.what() << std::endl;
    }  
    catch (const nlohmann::json::parse_error& e) {
      std::cerr << "3-message_arrived: Exception[parse_error] -" << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
      std::cerr << "3-message_arrived: Exception [logic_error]-" << e.what() << std::endl;
    } 
  }

  void delivery_complete(mqtt::delivery_token_ptr token) override {}

  bool isBrokerConnected() { return broker_connected_; }

  void setMQTTSubTopics(const std::vector<std::string>& mqtt_sub_topics, std::string ns) {
    mqtt_sub_topic_ns_ = ns;
    for (const auto& el : mqtt_sub_topics) {
      mqtt_sub_topics_.push_back(el);
    }
  }
  
  void setPayloadOutput(std::string payload_output_topic) {
    pub_from_client_ = nh_.advertise<std_msgs::String>(payload_output_topic, 10);
  }
  
  void setClientID(std::string client_id) { client_id_ = client_id; }

  std::string get_message() {
    std::lock_guard<std::mutex> lock(mtx_new_msg_flag_);
    if (new_msg_) {
      new_msg_ = false;
      return payload_str_msg_;
    } else
      return "-1";
  }
 
  void subscribeToMqttTopics(int view_type) {
    if (view_type == 0)  // Control
    {
      for (int i = 0; i < mqtt_sub_topics_.size(); i++) {
        std::string topic_name = mqtt_sub_topic_ns_ + mqtt_sub_topics_[i];
        cli_.subscribe(topic_name, COMMAND_QOS, nullptr, subListener_);
        subscribed_topics_.push_back(topic_name);
      }
    } else  // View
    {
      std::string topic_name = mqtt_sub_topic_ns_ + "breaklink";
      cli_.subscribe(topic_name, COMMAND_QOS, nullptr, subListener_);
      subscribed_topics_.push_back(topic_name);

      topic_name = mqtt_sub_topic_ns_ + "teardown";
      cli_.subscribe(topic_name, COMMAND_QOS, nullptr, subListener_);
      subscribed_topics_.push_back(topic_name);

      topic_name = mqtt_sub_topic_ns_ + "ping";
      cli_.subscribe(topic_name, COMMAND_QOS, nullptr, subListener_);
      subscribed_topics_.push_back(topic_name);
    }
  }

  void unSubscribeToMqttTopics(std::string topic_name = "") {
    if (topic_name != "") {
      cli_.unsubscribe(topic_name);
      ROS_INFO_STREAM("Unsubscribed from " << topic_name);
    } else {
      for (int i = 0; i < subscribed_topics_.size(); i++) {
        std::string topic_name = subscribed_topics_[i];

        cli_.unsubscribe(topic_name);
        ROS_INFO_STREAM("Unsubscribed from " << topic_name);
      }
    }
  }

  bool disconnect() {
    unSubscribeToMqttTopics();
    try {
      std::cout << "\nDisconnecting from the MQTT server..." << std::flush;
      cli_.disconnect()->wait();
      std::cout << "Disconnected from MQTT server.." << std::endl;
      return true;
    } catch (const mqtt::exception& exc) {
      std::cerr << "MQTT Server Disconnect error:" << exc << std::endl;
      return false;
    }
  }

 void sendOpsResponse(uint8_t operation_type, uint64_t timestamp, std::string reason)
  {
    bool errorRaised = false;
    bool operation_status = true; //accepted
    if (reason != "")
    {
       operation_status = false; //rejected
    }
    nlohmann::json ops_response;
    ops_response["topicName"] = "/remoteops/ops_response";
    ops_response["msgType"] = "ops_response";
    ops_response["reqReceivedTime"] = timestamp;
    ops_response["operationStatus"] = operation_status;
    ops_response["operationType"] = operation_type;
    ops_response["reason"] = reason;
    remoteops_ptr->serializeMsgFromVehicle(ops_response, errorRaised);

    if (errorRaised)
    {
      ROS_ERROR_STREAM("Error-serializing ops_response");
    }

  } 
  
 private:
  int nretry_;

  ros::Publisher pub_incoming_payload_, pub_from_client_;
  ros::NodeHandle nh_;
  // The MQTT client
  mqtt::async_client& cli_;
  // Options to use if we need to reconnect
  mqtt::connect_options& connOpts_;
  // An action listener to display the result of actions.
  action_listener subListener_;
  bool broker_connected_ = false;
  bool new_msg_ = false;
  std::string payload_str_msg_ = "";
  std::vector<std::string> mqtt_sub_topics_;
  std::vector<std::string> mqtt_pub_topics_;
  std::vector<std::string> subscribed_topics_;

  std::string client_id_;
  std::string mqtt_sub_topic_ns_ = "";
  std::string mqtt_pub_topic_ns_ = "";
};

class MqttRosInterface {
 public:
  MqttRosInterface(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    private_nh.param<std::string>("output_topic_client_payload", payload_output_topic_, "");
    private_nh.param<std::string>("intopic_client_payload", payload_input_topic_, "");
    user_id_ = std::getenv("USER");

    sub_to_client_ =
        nh.subscribe(payload_input_topic_, 10, &MqttRosInterface::remoteHandlerToMqttCallback, this,
                     ros::TransportHints().tcpNoDelay(true));

    // Publisher for remote handler
    pub_from_client_ = nh.advertise<std_msgs::String>(payload_output_topic_, 10);
  }

  bool isConnected() { return connected_; }
  std::string getPayloadOutputTopic() { return payload_output_topic_; };
  std::string getBrokerAddr() { return param_broker_addr_; };
  std::string getClientID() { return param_client_id_; };
  bool getDebugStatus() { return debug_; };
  void publish_from_client(std_msgs::String msg) { pub_from_client_.publish(msg); }

  void remoteHandlerToMqttCallback(const std_msgs::String::ConstPtr& msg) {
    mqtt_sub_topics_.clear();
    mqtt_pub_topics_.clear();

    nlohmann::json json_parsed_msg;

    try {
      json_parsed_msg = nlohmann::json::parse(msg->data);
    } catch (const std::exception& e) {
      std::cerr << "1- Error-remoteHandlerToMqttCallback[std]:" << e.what()<< std::endl;
    }
    catch (nlohmann::json::type_error& e) {
      std::cerr << "2- Error-remoteHandlerToMqttCallback[type_error]: " << e.what() << std::endl;
    }
    catch (const nlohmann::json::parse_error& e) {
      std::cerr << "3- Error-remoteHandlerToMqttCallback[parse_error]: " << e.what() << std::endl;
    }
    catch (const std::logic_error& e) {
      std::cerr << "4- Error-remoteHandlerToMqttCallback[logic_error]: " << e.what() << std::endl;
    }

    try
    {
    if (json_parsed_msg.contains("type")) {
      if (json_parsed_msg["type"] == "configuration") {
        param_client_id_ = json_parsed_msg["client_id"];
        param_broker_addr_ = json_parsed_msg["address"];
        timeout_sec_ = json_parsed_msg["timeout"];
        timeout_msec_ = round(1000 * timeout_sec_);
        mqtt_sub_topic_ns_ = json_parsed_msg["sub_ns"];
        mqtt_pub_topic_ns_ = json_parsed_msg["pub_ns"];
        debug_ = json_parsed_msg["debug"];
        // MQTTTopics that vehicle subscribes
        for (int i = 0; i < json_parsed_msg["sub_topic"].size(); i++) {
          std::string sub_topic_name = json_parsed_msg["sub_topic"][i]["mqtt_sub_topic_name"];
          if ((sub_topic_name != "") && (sub_topic_name != "linkup")) {
            mqtt_sub_topics_.push_back(sub_topic_name);
          }
        }

        connected_ = true;
        return;
      }
    }
    if (connected_) {
      // ROS_WARN_STREAM("json parsed msg:"<<json_parsed_msg);
      if (json_parsed_msg["msgType"] != "ego_state") {
        std::lock_guard<std::mutex> lock(mtx_to_client_vec);
        to_client_msg_vector_.push_back(json_parsed_msg);
      } 
      else 
      {
        std::lock_guard<std::mutex> lock(mtx_ego_state_vec);
        vec_ego_state.push_back(json_parsed_msg);
      }
    }
    }
    catch (const std::exception& e) {
      std::cerr << "5- Error-remoteHandlerToMqttCallback[std]:" << e.what()<< std::endl;
    }
    catch (nlohmann::json::type_error& e) {
      std::cerr << "6- Error-remoteHandlerToMqttCallback[type_error]: " << e.what() << std::endl;
    }
    catch (const nlohmann::json::parse_error& e) {
      std::cerr << "7- Error-remoteHandlerToMqttCallback[parse_error]: " << e.what() << std::endl;
    }
    catch (const std::logic_error& e) {
      std::cerr << "8- Error-remoteHandlerToMqttCallback[logic_error]: " << e.what() << std::endl;
    }
    
  }

  std::vector<std::string> mqtt_sub_topics_;
  std::vector<std::string> mqtt_pub_topics_;
  ros::Publisher pub_from_client_;                    // MQTT -> RemoteHandler
  ros::Subscriber sub_to_client_;                     // RemoteHandler -> MQTT
  std::vector<nlohmann::json> to_client_msg_vector_;  // MQTT publish messages

  int timeout_msec_;
  std::string mqtt_sub_topic_ns_ = "";
  std::string mqtt_pub_topic_ns_ = "";
  // MQTTROS
 private:
  std::string user_id_;
  std::string folder_path_;
  std::string mqtt_client_log_filepath_;

  std::string param_broker_addr_;
  std::string param_client_id_;
  bool connected_ = false;
  double timeout_sec_;
  std::ofstream statusMsgCsvFile_;

  std::string payload_output_topic_;
  std::string payload_input_topic_;

  bool debug_ = false;
};

void sendEgoStateToConsole() {
  
  
   
  while (ros::ok() && run_thread_ego_state.load()) 
  {

    try
    {
      if (remoteops_ptr)
      {
        
        if (!remoteops_ptr->getLinkupStatus())
        {
           continue;
        }
      }
      else 
      {
        continue;
      }


      int v_size = 0;
      {
         std::lock_guard<std::mutex> lock(mtx_ego_state_vec);
         v_size = vec_ego_state.size();
      }

      if (v_size > 0) 
      {
        nlohmann::json payload_from_vector;  
        
        {
          std::lock_guard<std::mutex> lock(mtx_ego_state_vec);
          payload_from_vector = vec_ego_state.front();
          vec_ego_state.erase(vec_ego_state.begin());
        }
        
        
        bool errorRaised = false;
        remoteops_ptr->serializeMsgFromVehicle(payload_from_vector, errorRaised);
        if (errorRaised)
        {
          ROS_ERROR_STREAM("Error-sendEgoStateToConsole msg: "<<payload_from_vector.dump());
        }
      }
    }
    catch (nlohmann::json::type_error& e) {
      std::cerr << "1-sendEgoStateToConsole: Exception[type_error]-"<< e.what() << std::endl;
    }
    catch (const std::exception& e) {
      std::cerr << "2-sendEgoStateToConsole: Exception[std]-" <<e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
      std::cerr << "1-sendEgoStateToConsole: Exception[logic_error]-"<< e.what() << std::endl;
    }
     catch (const nlohmann::json::parse_error& e) {
      std::cerr << "1-sendEgoStateToConsole: Exception[parse_error]-"<< e.what() << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  
    
  }
  std::cout<<"[remote_mqtt_client] thread(sendEgoStateToConsole) is shutting down..."<<std::endl;
}

void sendPingAckToConsole() {
  while (ros::ok() && run_thread_ping_ack.load()) {
    try
    {
      int v_size;
      {
        std::lock_guard<std::mutex> lock(mtx_console_ping_ack);
        v_size = vec_send_console_ping_ack.size();
      }

      if (v_size > 0) {
        nlohmann::json payload_from_vector;
        {
          std::lock_guard<std::mutex> lock(mtx_console_ping_ack);
          payload_from_vector = vec_send_console_ping_ack.front();
          vec_send_console_ping_ack.erase(vec_send_console_ping_ack.begin());
        
        }
        bool errorRaised = false;
        remoteops_ptr->serializeMsgFromVehicle(payload_from_vector, errorRaised);
        if (errorRaised)
        {
          ROS_ERROR_STREAM("Error-sendPingAckToConsole msg: "<< payload_from_vector.dump());
        }
      }
    }
    catch (nlohmann::json::type_error& e) {
      std::cerr << "1-sendPingAckToConsole: Exception[type_error]: "<< e.what() << std::endl;
    }
    catch (const std::exception& e) {
      std::cerr << "2-sendPingAckToConsole: Exception[std]: " << e.what() << std::endl;
    }  
    catch (const std::logic_error& e) {
      std::cerr << "3-sendPingAckToConsole: Exception[logic_error]: " << e.what() << std::endl;
    }  
    catch (const nlohmann::json::parse_error& e)  {
      std::cerr << "3-sendPingAckToConsole: Exception[parse_error]: " << e.what() << std::endl;
    }  
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout<<"[remote_mqtt_client] thread(sendPingAckToConsole) is shutting down..."<<std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "remoteops_mqtt_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  nlohmann::json json_status_msg;
  std_msgs::String status_str_msg;
  ros::Time status_msg_sent_time = ros::Time::now();

  ros::Rate r(100);
  MqttRosInterface mqtt_ros(nh, private_nh);

  while (!mqtt_ros.isConnected()) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_ERROR_STREAM("Remote Mqtt Client not connected to ROS handler... retrying in 1s ");
    ros::spinOnce();
    sleep(1);
  }

  mqtt::async_client mqtt_client_obj(mqtt_ros.getBrokerAddr(), mqtt_ros.getClientID());
  mqtt::connect_options connOpts;

  connOpts.set_mqtt_version(MQTTVERSION_5);

  int keep_alive_interval = 20;  // in seconds

  connOpts.set_clean_session(
      true);  // Disable message buffering to ensure messages are sent immediately.
  connOpts.set_keep_alive_interval(keep_alive_interval);

  callback cb(mqtt_client_obj, connOpts);

  mqtt_client_obj.set_callback(cb);
  std::shared_ptr<mqtt::async_client> mqtt_client_obj_ptr(&mqtt_client_obj);
  remoteops_ptr = std::make_shared<RemoteOps>(private_nh, mqtt_client_obj_ptr);

  cb.setClientID(mqtt_ros.getClientID());
  cb.setPayloadOutput(mqtt_ros.getPayloadOutputTopic());
  // Vehicle Subscribes MQTT Broker Remote Console topics
  cb.setMQTTSubTopics(mqtt_ros.mqtt_sub_topics_, mqtt_ros.mqtt_sub_topic_ns_);

  try {
    std::cout << "Connecting to the MQTT server...@ " << mqtt_ros.getBrokerAddr() << std::flush;
    mqtt_client_obj.connect(connOpts, nullptr, cb);
    std::cout << "MQTT Client Connected!" << std::endl;
    ROS_WARN_STREAM("mqtt Ver:" << connOpts.get_mqtt_version());

  } catch (const mqtt::exception& exc) {
    std::cerr << "\n[MQTT Client] ERROR: Unable to connect to MQTT server: '"
              << mqtt_ros.getBrokerAddr() << "'" << exc << std::endl;
  }

  json_status_msg["broker_status"] = cb.isBrokerConnected();
  status_str_msg.data = json_status_msg.dump();
  mqtt_ros.publish_from_client(status_str_msg);

  std::thread t_send_console_ping_ack(sendPingAckToConsole);
  std::thread t_send_console_ego_state(sendEgoStateToConsole);

  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();               // Start the spinner

  while (ros::ok()) {
    if (mqtt_ros.isConnected() && cb.isBrokerConnected()) {
      
      std_msgs::String payload_str_msg;
      int from_client_vec_size = 0;
      {
        std::lock_guard<std::mutex> lock(mtx_from_client_vec);
        from_client_vec_size = from_client_vec.size();
      }

      //payload_str_msg.data = cb.get_message();

    //  if (payload_str_msg.data != "-1") {
    //    mqtt_ros.publish_from_client(payload_str_msg);
      if ( from_client_vec_size > 0) 
      {
        std::lock_guard<std::mutex> lock(mtx_from_client_vec);
  
        try {
          //nlohmann::json payload_json_msg = nlohmann::json::parse(payload_str_msg.data);
          payload_str_msg.data = from_client_vec.front();
          from_client_vec.erase(from_client_vec.begin());
          mqtt_ros.publish_from_client(payload_str_msg);
        } 
        catch (const std::exception& e) {
          std::cerr << "1-remoteMqttClient[from_client_vec] Exception[std]:" <<"-"<< e.what()<<std::endl;
        }
        catch (const std::logic_error& e) {
          std::cerr << "2-remoteMqttClient[from_client_vec] Exception[logic_error]:" << e.what()<<std::endl;
        }
        catch (const nlohmann::json::parse_error& e) {
          std::cerr << "3-remoteMqttClient[from_client_vec] Exception[parse_error]:" << e.what()<<std::endl;
        }
        catch (const nlohmann::json::type_error& e) {
          std::cerr << "4-remoteMqttClient[from_client_vec] Exception[type_error]:" << e.what()<<std::endl;
        }
      }
      
      // Messages from ROS (remote handler)
      int v_size = 0;

      {
        std::lock_guard<std::mutex> lock(mtx_to_client_vec);
        v_size = mqtt_ros.to_client_msg_vector_.size();
      }


      if ( v_size > 0) {
       try
       {
        nlohmann::json payload_from_vector;
        
        {
          std::lock_guard<std::mutex> lock(mtx_to_client_vec);
          payload_from_vector = mqtt_ros.to_client_msg_vector_.front();
          mqtt_ros.to_client_msg_vector_.erase(mqtt_ros.to_client_msg_vector_.begin());
        }
        
        std::string msg_type = payload_from_vector["msgType"];
        std::string topic_name = payload_from_vector["topicName"];

        if (msg_type == "linkup") {
          ROS_WARN_STREAM("****LINKUP STATUS:" << payload_from_vector["resultText"]);

          if (payload_from_vector["resultText"] == "success") {
            int view_type = payload_from_vector["viewType"];

            ROS_WARN_STREAM("****Subscribing to topics*****");
            cb.subscribeToMqttTopics(view_type);
          }
        }

        remoteops_ptr->debug_ = mqtt_ros.getDebugStatus();
        // Sending message from vehicle to MQTT

        bool errorRaised = false; 
        remoteops_ptr->serializeMsgFromVehicle(payload_from_vector, errorRaised);
        if (errorRaised)
        {
          std::cerr<<"remoteMqttClientError - [main]serializeMsgFromVehicle:"<<payload_from_vector.dump()<<std::endl;
        }
       }
       catch (const std::exception& e) {
          std::cerr << "10 - remoteMqttClient Exception[std]:" << e.what()<<std::endl;
       }
       catch (const std::logic_error& e) {
          std::cerr << "20 - remoteMqttClient Exception[logic_error]:" << e.what()<<std::endl;
       }
       catch (const nlohmann::json::parse_error& e) {
          std::cerr << "30 - remoteMqttClient Exception[parse_error]:" << e.what()<<std::endl;
       }
       catch (const nlohmann::json::type_error& e) {
          std::cerr << "40 - remoteMqttClient Exception[type_error]:" << e.what()<<std::endl;
       }
      }

      if ((ros::Time::now() - status_msg_sent_time).toSec() > 5) {
        json_status_msg["broker_status"] = cb.isBrokerConnected();
        status_str_msg.data = json_status_msg.dump();
        mqtt_ros.publish_from_client(status_str_msg);
        status_msg_sent_time = ros::Time::now();
      }
      // ros::spinOnce();
      r.sleep();
    }
  
    else 
    {
      ROS_ERROR_THROTTLE(1.0, "[remote_mqtt_client] NOT CONNECTED!!");
    }
  
  }
  spinner.stop();
  //shutdown gracefully
  run_thread_ego_state.store(false);
  run_thread_ping_ack.store(false);
  if ( t_send_console_ping_ack.joinable() )
  {
    t_send_console_ping_ack.join();
  }
  if (t_send_console_ego_state.joinable()) 
  {
    t_send_console_ego_state.join();
  }
  cb.disconnect();

  return 0;
}
