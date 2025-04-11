#ifndef REMOTECONSOLE_H
#define REMOTECONSOLE_H

#include <remote_ops/remote_ops.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include <QMainWindow>
#include <QMessageBox>
#include <iostream>
#include <memory>

const int N_RETRY_ATTEMPTS = 5;

namespace Ui {
class MainWindow;
}

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
      : nretry_(1), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}

  bool broker_connected_ = false;
  bool mimic_remote_console_ = false;

  std::string linkup_sub_topic_, breaklink_sub_topic_, teardown_sub_topic_, ping_sub_topic_;

  // An action listener to display the result of actions.
  action_listener subListener_;

  std::shared_ptr<RemoteOps> remoteops_;

  void reconnect() {
    std::cout << "------------Remote Console - trying to reconect------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try {
      cli_.connect(connOpts_, nullptr, *this);
    } catch (const mqtt::exception& exc) {
      std::cerr << "Error: " << exc.what() << std::endl;
    }
  }

  // Re-connection failure
  void on_failure(const mqtt::token& tok) override {
    std::cout << "Remote Console - Connection attempt failed, retrying in 2s" << std::endl;
    broker_connected_ = false;
    sleep(2);
    reconnect();
  }

  // (Re)connection success
  void on_success(const mqtt::token& tok) override {}

  // (Re)connection success
  void connected(const std::string& cause) override {
    broker_connected_ = true;
    std::cout << std::endl << "REMOTE CONSOLE CONNECTED TO MQTT BROKER!" << std::endl;

    if (mimic_remote_console_)
    {
      if (linkup_sub_topic_ != "") {
        cli_.subscribe(linkup_sub_topic_, 2, nullptr, subListener_);
      }

      if (breaklink_sub_topic_ != "") {
        cli_.subscribe(breaklink_sub_topic_, 2, nullptr, subListener_);
      }

      if (teardown_sub_topic_ != "") {
        cli_.subscribe(teardown_sub_topic_, 2, nullptr, subListener_);
      }

      if (ping_sub_topic_ != "") {
        cli_.subscribe(ping_sub_topic_, 2, nullptr, subListener_);
      }
    }
  }

  // Callback for when the connection is lost.
  // This will initiate the attempt to manually reconnect.
  void connection_lost(const std::string& cause) override {
    std::cout << "\nRemote Console - Connection lost" << std::endl;
    if (!cause.empty()) std::cout << "\tcause: " << cause << std::endl;

    std::cout << "Remote Console - Reconnecting..." << std::endl;
    if (nretry_ < N_RETRY_ATTEMPTS) {
      std::cout << "\nMQTT Client Reconnecting [" << nretry_ << "/" << N_RETRY_ATTEMPTS
                << " attempt]" << std::endl;
      reconnect();
      nretry_++;
    } else {
      std::cout << "\nMQTT Client - Retry attemtps..." << std::endl;
    }
  }

  // Callback for when a message arrives.
  void message_arrived(mqtt::const_message_ptr msg) override 
  {
      if (!mimic_remote_console_){ return; }
      uint64_t currentTimestamp = remoteops_->getTimestamp();
      std::string topic_name = msg->get_topic();
      nlohmann::json deserialized_msg;
      
      if (topic_name == "/remoteops/tf") 
      {
        return;
      }
      if (topic_name == "/remoteops/ego_state") 
      {
         remoteops_->deserializeMsgFromVehicle(msg, deserialized_msg);
         ROS_WARN_STREAM("received ego_state"<<deserialized_msg.dump());
      }
     
      if (topic_name == "/remoteops/ping") 
      {
          remoteops_->deserializeMsgFromVehicle(msg, deserialized_msg);
         
          if (!deserialized_msg.contains("latency") && (deserialized_msg["receiver"] == remoteops_->getConsoleID()))
          {
           
            // std::string msg_out = "*Console is sending PING ACK message";
            // remoteops_->ROS_INFO_STREAM_COLOR("cyan", msg_out);
            nlohmann::json payload;
            std::string sender = remoteops_->getConsoleID();
            std::string receiver = remoteops_->getVehicleID();
            uint8_t seq =deserialized_msg["seq"];
            uint64_t requestLatency = 1; //dummy

            payload["topicName"] = topic_name;
            payload["msgType"] = "consolePingAck";
            payload["sender"] = sender;
            payload["receiver"] = receiver;
            payload["seq"] = seq;
            payload["requestLatency"] = requestLatency;//request_latency_ms;
            bool errorRaised = false;
            remoteops_->serializeMsgFromVehicle(payload, errorRaised);
          }
      } 
      else if (topic_name == "/remoteops/breaklink") 
      {
         ROS_WARN_STREAM("RemoteConsole-message-arrived:"<<topic_name);

         remoteops_->deserializeMsgFromVehicle(msg, deserialized_msg);
         //Send console breaklink ACK
         if (deserialized_msg.contains("breakLinkReason") && !isConsoleBreaklinkRequest_)
         {
              nlohmann::json payload;
              payload["topicName"] = topic_name;
              payload["msgType"] = "consoleBreaklinkAck";
              payload["consoleID"] = deserialized_msg["consoleID"];
              payload["vehicleID"] = deserialized_msg["vehicleID"];
              bool errorRaised = false;
              remoteops_->serializeMsgFromVehicle(payload, errorRaised);
         } 
         else if (!deserialized_msg.contains("breakLinkReason") && isConsoleBreaklinkRequest_)
         {
           ROS_WARN_STREAM("Remote Console doing NOTHING - VEHICLE ACK!!!");
           isConsoleBreaklinkRequest_ = false;
         }
       
      }
      else 
      {
        ROS_WARN_STREAM("RemoteConsole-message-arrived:"<<topic_name);
      }
  
  
  
  }

  void delivery_complete(mqtt::delivery_token_ptr token) override {}

  std::string getMessage() {

    if (new_msg_) {
      new_msg_ = false;
      return payload_str_msg_;
    } else
      return "-1";
  }

 std::string payload_str_msg_ = "";
 nlohmann::json payload_json_msg_;
 bool isConsoleBreaklinkRequest_ = false;

 private:
  // Counter for the number of connection retries
  int nretry_;
  // The MQTT client
  mqtt::async_client& cli_;
  // Options to use if we need to reconnect
  mqtt::connect_options& connOpts_;
  bool new_msg_ = false;

};

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = 0);

  void initialize();
  void getParams();
  void mqttConnect();
  void mqttDisconnect();
  void sendReq(std::string topic);
  ~MainWindow();

 private:
  Ui::MainWindow* ui;
  ros::NodeHandle private_nh_;
  std::string console_id_, vehicle_id_, broker_addr_;
  std::string linkup_pub_topic_, breaklink_pub_topic_, teardown_pub_topic_, ping_pub_topic_;
  std::string linkup_sub_topic_, breaklink_sub_topic_, teardown_sub_topic_, ping_sub_topic_;
  std::string sub_topic_namespace_;
  std::string pub_topic_namespace_;
  std::shared_ptr<mqtt::async_client> cli_;
  std::shared_ptr<callback> cb_;
  std::shared_ptr<RemoteOps> remoteops_;
  mqtt::connect_options connOpts_;
  bool mimic_remote_console_ = false;
  bool isEmergencyBrake_  = false;
  bool isHazardLightsOn_  = false;
  uint8_t nudgeInstanceId_ = 0; 

  //set destination
  std::string block_name_;
  int block_id_, lane_id_, slot_id_, container_type_, job_type_;
  
  //non-yard goal
  int destination_id_;
  
  //move by distance
  double move_by_distance_;

};

#endif  // MAINWINDOW_H
