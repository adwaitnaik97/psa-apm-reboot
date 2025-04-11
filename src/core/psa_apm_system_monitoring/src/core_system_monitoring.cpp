#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <aios_apm_msgs/Status.h>
#include <apm_diagnostic_msgs/TopicListDiagnostic.h>
#include <apm_diagnostic_msgs/NodeListDiagnostic.h>
#include "std_msgs/Float32.h"

class core_system_monitor
{
public:
  core_system_monitor()
  {
    topic_monitor_sub = nh.subscribe("/aisd/apm_diagnostic/core/topics", 10, &core_system_monitor::topic_monitor_Callback, this);
    node_monitor_sub = nh.subscribe("/aisd/apm_diagnostic/core/nodes", 10, &core_system_monitor::node_monitor_Callback, this);
    core_system_status_pub = nh.advertise<aios_apm_msgs::Status>("/aisd/system_monitoring/core_result", 10);
    core_action_pub = nh.advertise<std_msgs::Float32>("/aisd/actions/apm_core_diagnostic", 5);

    core_node_status.v2e_error = true;
    core_node_status.v2e_timeout = true;
    core_node_status.v2i_error = true;
    core_node_status.v2i_timeout = true;
    core_node_status.avcs_error = true;
    core_node_status.avcs_timeout = true;
    core_node_status.teleops_error = true;
    core_node_status.teleops_timeout =  true;
    core_node_status.alignment_control_timeout = true;
    core_node_status.alignment_control_error = true;
    core_node_status.tl_timeout = true;
    core_node_status.tl_error = true;
    core_node_status.ontology_timeout = true;
    core_node_status.ontology_error = true;
    core_node_status.aios_preprocessing_timeout = true;
    core_node_status.aios_preprocessing_error = true;
    core_node_status.session_management_timeout = true;
    core_node_status.session_management_error = true;
    core_node_status.session_summarizer_timeout = true;
    core_node_status.session_summarizer_error = true;
    // core_node_status.remote_handler_timeout = true
    core_node_status.remote_handler_error = true;
    core_node_status.behaviour_summarizer_timeout = true;
    core_node_status.behaviour_summarizer_error = true;

  }

  void topic_monitor_Callback(const apm_diagnostic_msgs::TopicListDiagnostic::ConstPtr msg)
  {
    core_node_status.header.stamp = ros::Time::now();
    // core_node_status.v2e_timeout = bool(!msg->topics[1].flag);
    core_node_status.v2i_timeout = bool(!msg->topics[2].flag);
    core_node_status.avcs_timeout = bool(!msg->topics[9].flag);
    core_node_status.alignment_control_timeout = bool(!msg->topics[8].flag);
    core_node_status.tl_timeout = bool(!msg->topics[10].flag);
    core_node_status.ontology_timeout = bool(!msg->topics[11].flag);
    core_node_status.aios_preprocessing_timeout = bool(!msg->topics[12].flag);
    core_node_status.session_management_timeout = bool(!msg->topics[13].flag);
    core_node_status.session_summarizer_timeout = bool(!msg->topics[14].flag);
    core_node_status.behaviour_summarizer_timeout = bool(!msg->topics[15].flag);
    // core_node_status.remote_handler_timeout = bool(!msg->topics[2].flag);

    core_system_status_pub.publish(core_node_status);
    coreStatusPublisher();
  }

  void node_monitor_Callback(const apm_diagnostic_msgs::NodeListDiagnostic::ConstPtr msg)
  {
    // core_node_status.v2e_error = bool(!msg->nodes[1].flag);
    core_node_status.avcs_error = bool(!msg->nodes[1].flag);
    core_node_status.v2i_error = bool(!msg->nodes[2].flag);
    core_node_status.alignment_control_error = bool(!msg->nodes[8].flag);
    core_node_status.tl_error = bool(!msg->nodes[10].flag);
    core_node_status.ontology_error = bool(!msg->nodes[11].flag);
    core_node_status.aios_preprocessing_error = bool(!msg->nodes[12].flag);
    core_node_status.session_management_error = bool(!msg->nodes[13].flag);
    core_node_status.session_summarizer_error = bool(!msg->nodes[14].flag);
    core_node_status.remote_handler_error = bool(!msg->nodes[15].flag);
    core_node_status.behaviour_summarizer_error = bool(!msg->nodes[16].flag);
    

  }

  void coreStatusPublisher()
  {
    float action_value = 3.0;
    if(core_node_status.avcs_error || core_node_status.v2i_error || core_node_status.alignment_control_error || core_node_status.tl_error || core_node_status.ontology_error || core_node_status.aios_preprocessing_error || core_node_status.session_management_error || core_node_status.session_summarizer_error || core_node_status.remote_handler_error || core_node_status.behaviour_summarizer_error)
    {
      action_value = 1.0;
    }
    else if(core_node_status.v2i_timeout || core_node_status.avcs_timeout || core_node_status.alignment_control_timeout || core_node_status.tl_timeout || core_node_status.ontology_timeout || core_node_status.aios_preprocessing_timeout || core_node_status.session_management_timeout || core_node_status.session_summarizer_timeout || core_node_status.behaviour_summarizer_timeout)
    {
      action_value = 2.0;
    }
   

    std_msgs::Float32 core_action;
    core_action.data = action_value;
    core_action_pub.publish(core_action);
  }

private:
  ros::NodeHandle nh;
  ros::Publisher core_system_status_pub;
  ros::Subscriber topic_monitor_sub, node_monitor_sub;
  aios_apm_msgs::Status core_node_status;
  ros::Publisher core_action_pub;
};
// End of class

int main(int argc, char **argv)
{
  ros::init(argc, argv, "core_system_monitor");
  core_system_monitor core_system_monitor_object;

  ros::spin();
  return 0;
}
