#include "remote_ops/remote_console.h"

#include "ui_remote_console.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), private_nh_("~") {
  ui->setupUi(this);

  this->setWindowTitle("Remote Console Operations");
  this->setFixedSize(size());

  getParams();
  initialize();

  mqttConnect();
  while (!cb_->broker_connected_) {
    ui->label_status_msg->setText(QString::fromStdString("Trying to connect to MQTT Broker..."));
  }
  
  ui->label_status_msg->setText(QString::fromStdString("Connected to MQTT Broker..."));
  ui->pB_linkup_sendReq->setEnabled(true);
  ui->pB_breaklink_sendReq->setEnabled(true);
  ui->pB_teardown_sendReq->setEnabled(true);
  ui->pB_ping_sendReq->setEnabled(true);

  ui->pB_emergencyBrake_sendReq->setEnabled(true);
  ui->pB_horn_sendReq->setEnabled(true);
  ui->pB_headLight_sendReq->setEnabled(true);
  ui->pB_indicatorLight_sendReq->setEnabled(false);
  ui->pB_fogLight_sendReq->setEnabled(false);
  ui->pB_ignitionCmd_sendReq->setEnabled(false);
  ui->pB_hazardLight_sendReq->setEnabled(true);
  ui->pB_setDestination_sendReq->setEnabled(true);
  ui->pB_reLocalization_sendReq->setEnabled(true);
  ui->pB_adjustPosition_sendReq->setEnabled(true);
  ui->pB_egoState_sendReq->setEnabled(true);
  ui->pB_manualPushRemote_sendReq->setEnabled(true);
  ui->pB_trajectoryOverride_sendReq->setEnabled(true);
  ui->pB_nonyardGoal_sendReq->setEnabled(true);
  ui->pB_pauseResume_sendReq->setEnabled(true);
  ui->pB_gearCmd_sendReq->setEnabled(false);
  ui->pB_workingLight_sendReq->setEnabled(false);
  ui->pB_acControl_sendReq->setEnabled(false);
  ui->pB_precedenceOverride_sendReq->setEnabled(true);
  ui->pB_completeJob_sendReq->setEnabled(false);
  ui->pB_trafficLightOverride_sendReq->setEnabled(true);
  ui->pB_weatherStatus_sendReq->setEnabled(false);
  ui->pB_lightStatus_sendReq->setEnabled(false);
}

void MainWindow::initialize() {
  QObject::connect(ui->pB_linkup_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("linkup"); });
  QObject::connect(ui->pB_breaklink_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("breaklink"); });
  QObject::connect(ui->pB_teardown_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("teardown"); });
  QObject::connect(ui->pB_ping_sendReq, &QPushButton::clicked, this, [=]() { sendReq("ping"); });
  QObject::connect(ui->pB_emergencyBrake_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("emergency_brake_command"); });
  QObject::connect(ui->pB_horn_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_horn"); });
  QObject::connect(ui->pB_headLight_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_head_light_remote"); });
  QObject::connect(ui->pB_fogLight_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_fog_light"); });
  QObject::connect(ui->pB_hazardLight_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_signal_light"); });
  QObject::connect(ui->pB_setDestination_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("goal"); });
  QObject::connect(ui->pB_reLocalization_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("location"); });
  QObject::connect(ui->pB_adjustPosition_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("adjust_position"); });
  QObject::connect(ui->pB_egoState_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("ego_state"); });
  QObject::connect(ui->pB_manualPushRemote_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("manual_push_remote"); });
  QObject::connect(ui->pB_trajectoryOverride_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("override_path"); });
  QObject::connect(ui->pB_nonyardGoal_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("non_yard_goal"); });
  QObject::connect(ui->pB_pauseResume_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("stop"); });
  QObject::connect(ui->pB_gearCmd_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_gear"); });
  QObject::connect(ui->pB_ignitionCmd_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_ignition"); });
  QObject::connect(ui->pB_workingLight_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_working_light"); });
  QObject::connect(ui->pB_acControl_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_ac_control"); });
  QObject::connect(ui->pB_precedenceOverride_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("precedence_override"); });
  QObject::connect(ui->pB_completeJob_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_complete_job"); });
  QObject::connect(ui->pB_trafficLightOverride_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("traffic_light_override"); });
  QObject::connect(ui->pB_lightStatus_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_light_state"); });
  QObject::connect(ui->pB_weatherStatus_sendReq, &QPushButton::clicked, this,
                   [=]() { sendReq("cmd_weather_state"); });

  QObject::connect(ui->pB_close, &QPushButton::clicked, this, [=]() { close(); });

  ui->pB_linkup_sendReq->setEnabled(false);
  ui->pB_breaklink_sendReq->setEnabled(false);
  ui->pB_teardown_sendReq->setEnabled(false);
  ui->pB_ping_sendReq->setEnabled(false);
  ui->pB_emergencyBrake_sendReq->setEnabled(false);
  ui->pB_horn_sendReq->setEnabled(false);
  ui->pB_headLight_sendReq->setEnabled(false);
  ui->pB_indicatorLight_sendReq->setEnabled(false);
  ui->pB_fogLight_sendReq->setEnabled(false);
  ui->pB_hazardLight_sendReq->setEnabled(false);
  ui->pB_setDestination_sendReq->setEnabled(false);
  ui->pB_reLocalization_sendReq->setEnabled(false);
  ui->pB_adjustPosition_sendReq->setEnabled(false);
  ui->pB_egoState_sendReq->setEnabled(false);
  ui->pB_manualPushRemote_sendReq->setEnabled(false);
  ui->pB_trajectoryOverride_sendReq->setEnabled(false);
  ui->pB_nonyardGoal_sendReq->setEnabled(false);
  ui->pB_pauseResume_sendReq->setEnabled(false);
  ui->pB_gearCmd_sendReq->setEnabled(false);
  ui->pB_ignitionCmd_sendReq->setEnabled(false);
  ui->pB_workingLight_sendReq->setEnabled(false);
  ui->pB_acControl_sendReq->setEnabled(false);
  ui->pB_completeJob_sendReq->setEnabled(false);
  ui->pB_trafficLightOverride_sendReq->setEnabled(false);
  ui->pB_weatherStatus_sendReq->setEnabled(false);
  ui->pB_lightStatus_sendReq->setEnabled(false);

  cli_ = std::make_shared<mqtt::async_client>(broker_addr_, console_id_);
  remoteops_ = std::make_shared<RemoteOps>(private_nh_, cli_);

  cb_ = std::make_shared<callback>(*cli_, connOpts_);
  cb_->remoteops_ = remoteops_;
  cb_->linkup_sub_topic_ = sub_topic_namespace_ + linkup_sub_topic_;
  cb_->teardown_sub_topic_ = sub_topic_namespace_ + teardown_sub_topic_;
  cb_->ping_sub_topic_ = sub_topic_namespace_ + ping_sub_topic_;
  cb_->breaklink_sub_topic_ = sub_topic_namespace_ + breaklink_sub_topic_;
  cb_->mimic_remote_console_ = mimic_remote_console_;
  cli_->set_callback(*cb_);

  ui->comboBox_trafficLightOverride->setCurrentIndex(0);

  // Disable AMBER index = 2
  ui->comboBox_trafficLightOverride->setItemData(2, QVariant(0), Qt::UserRole - 1);
}

void MainWindow::getParams() {
  private_nh_.getParam("mimic_remote_console", mimic_remote_console_);

  private_nh_.getParam("console_id", console_id_);
  private_nh_.getParam("vehicle_id", vehicle_id_);
  private_nh_.getParam("mqtt_broker_addr", broker_addr_);

  // std::string namespace_str = private_nh_.getNamespace();
  // ROS_INFO("Namespace of private NodeHandle: %s", namespace_str.c_str());

  private_nh_.param<std::string>("pub_topic_namespace", pub_topic_namespace_, std::string("/"));
  private_nh_.param<std::string>("sub_topic_namespace", sub_topic_namespace_, std::string("/"));

  // Published topics
  private_nh_.param<std::string>("linkup_pub_topic", linkup_pub_topic_, std::string("/"));
  private_nh_.param<std::string>("breaklink_pub_topic", breaklink_pub_topic_, std::string("/"));
  private_nh_.param<std::string>("teardown_pub_topic", teardown_pub_topic_, std::string("/"));
  private_nh_.param<std::string>("ping_pub_topic", ping_pub_topic_, std::string("/"));

  // Subscribed Topics
  private_nh_.param<std::string>("linkup_sub_topic", linkup_sub_topic_, std::string("/"));
  private_nh_.param<std::string>("teardown_sub_topic", teardown_sub_topic_, std::string("/"));
  private_nh_.param<std::string>("breaklink_sub_topic", breaklink_sub_topic_, std::string("/"));
  private_nh_.param<std::string>("ping_sub_topic", ping_sub_topic_, std::string("/"));




  ui->lineEdit_consoleID->setText(QString::fromStdString(console_id_));
  ui->lineEdit_vehicleID->setText(QString::fromStdString(vehicle_id_));
  ui->lineEdit_brokerIP->setText(QString::fromStdString(broker_addr_));
  ui->lineEdit_consoleID->setReadOnly(true);
  ui->lineEdit_vehicleID->setReadOnly(true);
  ui->lineEdit_brokerIP->setReadOnly(true);
}

void MainWindow::mqttConnect() {
  connOpts_.set_clean_session(true);
  connOpts_.set_mqtt_version(MQTTVERSION_5);
  try {
    std::cout << "Connecting to the MQTT server...@ " << broker_addr_ << std::flush;
    cli_->connect(connOpts_, nullptr, *cb_);

  } catch (const mqtt::exception& exc) {
    std::cerr << "\nERROR: Remote Console - unable to connect to MQTT server: '" << broker_addr_
              << "'" << exc << std::endl;
  }
}

void MainWindow::mqttDisconnect() {
  //  std::this_thread::sleep_for(std::chrono::seconds(5));
  cli_->disconnect()->wait();
  std::cout << "Disconnected" << std::endl;
}

void MainWindow::sendReq(std::string topic) {
  QMessageBox msgBox;
  nlohmann::json payload;

  std::string topic_name_with_ns = pub_topic_namespace_ + topic;
  std::string type = topic;

  if (ui->lineEdit_consoleID->text().toStdString() == "") {
    msgBox.setText(QString::fromStdString("Console ID is required!"));
    msgBox.exec();
    return;
  }

  if (ui->lineEdit_vehicleID->text().toStdString() == "") {
    msgBox.setText(QString::fromStdString("Vehicle ID is required!"));
    msgBox.exec();
    return;
  }

  if (ui->lineEdit_brokerIP->text().toStdString() == "") {
    msgBox.setText(QString::fromStdString("Broker address is required!"));
    msgBox.exec();
    return;
  }
 
  if (cb_->broker_connected_) {
    if (topic == "linkup") {
      remoteops_->setVehicleID(ui->lineEdit_vehicleID->text().toStdString());
      remoteops_->setConsoleID(ui->lineEdit_consoleID->text().toStdString());
 
      std::string view_type = ui->comboBox_linkup_viewType->currentText().toStdString();
      std::string console_mode = ui->comboBox_linkup_consoleMode->currentText().toStdString();
      remoteops_->setViewType(view_type);
      remoteops_->setConsoleMode(console_mode);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "breaklink") {
      std::string breaklink_reason = ui->comboBox_breaklink_reason->currentText().toStdString();
      remoteops_->setBreaklinkReason(breaklink_reason);
      cb_->isConsoleBreaklinkRequest_ = true;
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "ping") {
      // while (ros::ok())
      // {
        // auto start_time = std::chrono::high_resolution_clock::now();
        remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
        // auto end_time = std::chrono::high_resolution_clock::now();
      //   int64_t latency = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
      // //   sleep(0.1);
      // }
     
    } else if (topic == "teardown") {
      ROS_WARN_STREAM("CONSOLE Sending teardown message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "emergency_brake_command") {
      isEmergencyBrake_ = !isEmergencyBrake_;
      ROS_WARN_STREAM("CONSOLE Sending emergency_brake_command message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type, isEmergencyBrake_); 
    } else if (topic == "cmd_horn") {
      ROS_WARN_STREAM("CONSOLE Sending horn message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "cmd_signal_light") {
      isHazardLightsOn_ = !isHazardLightsOn_;
      ROS_WARN_STREAM("CONSOLE Sending cmd_signal_light message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type, isEmergencyBrake_, isHazardLightsOn_); 
    } else if (topic == "goal") {
      ROS_WARN_STREAM("CONSOLE Sending goal message");

     private_nh_.param<std::string>("block_name", block_name_, std::string("TD"));
     private_nh_.param<int>("block_id", block_id_, 2);
     private_nh_.param<int>("lane_id", lane_id_, 11);
     private_nh_.param<int>("slot_id", slot_id_, 28);
     private_nh_.param<int>("container_type", container_type_, 1);
     private_nh_.param<int>("job_type", job_type_, 1);
 
 

      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "location") {
      ROS_WARN_STREAM("CONSOLE Sending location message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "manual_push_remote") {
      ROS_WARN_STREAM("CONSOLE Sending manual_push_remote message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type, false, false, nudgeInstanceId_);
      nudgeInstanceId_++; 
    } else if (topic == "override_path") {
      ROS_WARN_STREAM("CONSOLE Sending override_path message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "non_yard_goal") {
      ROS_WARN_STREAM("CONSOLE Sending non_yard_goal message");
      private_nh_.param<int>("destination_id", destination_id_, 18);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "adjust_position") {
      ROS_WARN_STREAM("CONSOLE Sending adjust_position message");
      private_nh_.param<double>("move_by_distance", move_by_distance_, 2.0);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "cmd_complete_job") {
      ROS_WARN_STREAM("CONSOLE Sending cmd_complete_job message");
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type); 
    } else if (topic == "cmd_head_light_remote") {
      int state = ui->comboBox_headLight->currentIndex();
      std::cout << "State:" << state << std::endl;
      remoteops_->setHeadLightState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);

    } 
    else if (topic == "cmd_fog_light") {
      std::string txt = ui->pB_fogLight_sendReq->text().toStdString();
      bool state = false;

      if (txt == "Fog Light On") {
        ui->pB_fogLight_sendReq->setText("Fog Light Off");
        state = true;
      } else if (txt == "Fog Light Off") {
        ui->pB_fogLight_sendReq->setText("Fog Light On");
        state = false;
      }
      remoteops_->setFogLightState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);

    } else if (topic == "cmd_ignition") {
      std::string txt = ui->pB_ignitionCmd_sendReq->text().toStdString();
      bool state = false;

      if (txt == "Ignition On") {
        ui->pB_ignitionCmd_sendReq->setText("Ignition Off");
        state = true;
      } else if (txt == "Ignition Off") {
        ui->pB_ignitionCmd_sendReq->setText("Ignition On");
        state = false;
      }
      remoteops_->setIgnitionState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);

    } else if (topic == "cmd_gear") {
      int state = ui->comboBox_gearCmd->currentIndex();
      remoteops_->setGearState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "cmd_working_light") {
      bool rear_state = ui->checkBox_wlRear->isChecked();
      bool front_state = ui->checkBox_wlFront->isChecked();
      remoteops_->setWorkingLight(rear_state, front_state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "cmd_ac_control") {
      std::string txt = ui->pB_acControl_sendReq->text().toStdString();
      bool state = false;

      if (txt == "AC On") {
        ui->pB_acControl_sendReq->setText("AC Off");
        state = true;
      } else if (txt == "AC Off") {
        ui->pB_acControl_sendReq->setText("AC On");
        state = false;
      }
      remoteops_->setAcState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "stop") {
      std::string state = ui->pB_pauseResume_sendReq->text().toStdString();
      std::cout << "State:" << state << std::endl;
      bool pause;
      if (state == "Pause") {
        ui->pB_pauseResume_sendReq->setText("Resume");
        pause = true;
      } else if (state == "Resume") {
        ui->pB_pauseResume_sendReq->setText("Pause");
        pause = false;
      }
      remoteops_->setPauseResume(pause);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "traffic_light_override") {
     
      int state = ui->comboBox_trafficLightOverride->currentIndex();
      std::cout << "TL State:" << state << std::endl;
      remoteops_->setTrafficLightOverrideState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "cmd_weather_state") {
      int state = ui->comboBox_weatherStatus->currentIndex();
      remoteops_->setWeatherState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "cmd_light_state") {
      int state = ui->comboBox_lightStatus->currentIndex();
      remoteops_->setLightState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "precedence_override") {
      bool state = ui->comboBox_precedenceOverride->currentIndex();
      remoteops_->setPrecedenceOverrideState(state);
      remoteops_->serializeMsgFromConsole(topic_name_with_ns, type);
    } else if (topic == "ego_state") {
    payload["topicName"] = "/remoteops/ego_state";
    payload["msgType"] = "ego_state";
    payload["signalLight"] = ui->lineEdit_egoSignalLight->text().toInt();
    payload["vehPositionX"] = ui->lineEdit_egoVehPositionX->text().toFloat();
    payload["vehPositionY"] = ui->lineEdit_egoVehPositionY->text().toFloat();
    payload["vehYaw"] = ui->lineEdit_egoVehYaw->text().toFloat();
    payload["trailerPositionX"] = ui->lineEdit_egoTrailerPositionX->text().toFloat();
    payload["trailerPositionY"] = ui->lineEdit_egoTrailerPositionY->text().toFloat();
    payload["trailerYaw"] = ui->lineEdit_egoTrailerYaw->text().toFloat();
    payload["gearStatus"] = ui->lineEdit_egoGearStatus->text().toInt();
    payload["brakePercentage"] = ui->lineEdit_egoBrakePercentage->text().toFloat();
    payload["velocity"] = ui->lineEdit_egoVelocity->text().toFloat();
    payload["steeringAngle"] = ui->lineEdit_egoSteeringAngle->text().toFloat();
    bool remoteEmergencyButtonStatus = false;
    if (ui->lineEdit_egoRemoteEmergencyButtonStatus->text().toInt() == 1) {
      remoteEmergencyButtonStatus = true;
    }
    payload["remoteEmergencyButtonStatus"] = remoteEmergencyButtonStatus;
    payload["localizationStatus"] = ui->lineEdit_egoLocalizationStatus->text().toInt();
    payload["headLights"] = ui->lineEdit_egoHeadLights->text().toInt();
    payload["throttlePercentage"] = ui->lineEdit_egoThrottlePercentage->text().toFloat();

    bool hornCmdFeedback = false;
    if (ui->lineEdit_egoHornCmdFeedback->text().toInt() == 1) {
      hornCmdFeedback = true;
    }
    payload["hornCmdFeedback"] = hornCmdFeedback;

    bool inSSA = false;
    if (ui->lineEdit_egoInSSA->text().toInt() == 1) {
      inSSA = true;
    }
    payload["inSSA"] = inSSA;
    bool errorRaised = false; 
    remoteops_->serializeMsgFromVehicle(payload, errorRaised);
    // ROS_WARN_STREAM("Vehicle serializing ego_state");

  }
  }

}

MainWindow::~MainWindow() {
  if (cli_) {
    this->mqttDisconnect();
  }

  delete ui;
}
