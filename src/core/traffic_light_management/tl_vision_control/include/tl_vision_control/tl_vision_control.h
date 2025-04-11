#ifndef PROJECT_tl_vision_control_H
#define PROJECT_tl_vision_control_H

#define __APP_NAME__ "tl_vision_control"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "aipe_msgs/DetectedObjectArray.h"
#include "airs_msgs/reg37.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <aisc_msgs/SituationCognition.h>
#include <traffic_light_common/tl_types.h>
#include "aipe_msgs/ClassificationArray.h"
#include <aipe_msgs/TrafficLightLog.h>
#include "aifo_client/TripJobStatus.h"
#include <chrono>

namespace aipe
{
    class JunctionStateEstimator;
    class CASStateEstimator;
    class TrafficLightControlApp
    {

    public:
        void Run();

        /*!
        * ROS Node to determine and publish junction state in terms of traffic lights
        * 
        * APM's lane id, light indicator, top centre camera frame, fused objects are used as input. Traffic light rules for each junction and lane, 
        * utm coordinates of traffic light poles are loaded to determine the state of the junction considering all this information.
        * Status of all traffic lights belonging to the current junction is being published with their utm coordinates. 
        * See "tl_vision_control.launch" file for more details.
        */
        TrafficLightControlApp();

        /*!
        * Callback to get fused objects
        * @param in_image_msg ros message
        */
        void DetectionsCallback(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg);

        /*!
        * Callback to get classification results of front top centre camera
        * @param in_classifications ros message
        */
        void ClassificationsFTCCallback(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications);

        /*!
        * Callback to get classification results of front bottom left camera
        * @param in_classifications ros message
        */
        void ClassificationsFBLCallback(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications);

        /*!
        * Callback to get classification results of front bottom right camera
        * @param in_classifications ros message
        */
        void ClassificationsFBRCallback(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications);

        /*!
        * Convert all given objects to markers for visualization on Rviz.
        * @param in_objects traffic light objects
        * @param final_state final state of the junction ("Red", "Amber", "Green")
        * @return markers
        */
        visualization_msgs::MarkerArray ObjectsToMarkers(const aipe_msgs::DetectedObjectArray &in_objects, const std::string &final_state);
        visualization_msgs::MarkerArray ObjectsToMarkers(const aipe_msgs::DetectedObjectArray &in_objects);

        /*!
        * Reads the config params from the command line
        * @param in_private_handle
        */
        void InitializeRosIo(ros::NodeHandle &in_private_handle);

        /*!
        * Callback to get current apm speed. Used in both speed estimation and traffic light localization.
        * @param in_apm_speed ros message
        */
        void SpeedCallback(const airs_msgs::reg37::ConstPtr &in_apm_speed);

        /*!
        * Callback to get current lane of APM. Used in decision of junction status regarding the traffic lights
        * @param in_lane ros message
        */
        void LaneCallback(const std_msgs::String::ConstPtr &in_lane);

        /*!
        * Callback to get light indicator representing the next movement of APM
        * @param in_light_indicator ros message
        */
        void LightIndicatorCallback(const std_msgs::Int64::ConstPtr &in_light_indicator);

        /*!
        * Callback to get hazard situation representing availability of the road on right-turn. 
        * This flag is used to give right-turn signal while the straight is green and road is empty
        * @param in_hazard ros message
        */
        void HazardCallback(const aisc_msgs::SituationCognition::ConstPtr &in_hazard);

        /*!
        * Callback to get target location id of CAS. Used in decision of which camera is being used for traffic light recognition
        * @param in_trip_job_status ros message
        */
        void TripJobCallback(const aifo_client::TripJobStatus::ConstPtr &in_trip_job_status);

        /*!
        * Callback to apm status. It's used to set a timer to end CAS traffic light reading after a predefined time in the alignment job
        * @param in_apm_status ros message
        */
        void ApmStatusCallback(const std_msgs::Int64::ConstPtr &in_apm_status);

        /*!
        * Callback to get top centre camera image width
        * @param in_message ros message
        */
        void CameraInfoCallback(const sensor_msgs::CameraInfo &in_message);

        /*!
        * Callback to get junction id (ppt09, ppt10, etc.)
        * @param in_message ros message
        */
        void JunctionIdCallback(const std_msgs::String::ConstPtr in_msg);

        /*!
        * This method is called if the APM is at a junction.
        * @param in_msg detections
        * @param final_state final state of the traffic light junction
        * @param state_conf State confidence, 0.0 to 1.0 
        */
        void JunctionAnalysis(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg, std::string &final_state, float &state_conf);

        /*!
        * This method is called if the APM is in a junction.
        * @param in_msg detections
        */
        void BayAnalysis(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg);

        /*!
        * Filter out the classification results that are not related with CAS traffic light system
        * @param in_classifications received classification
        * @param filtered_classifications filtered results
        */
        void FilterClassifications(const aipe_msgs::ClassificationArray::ConstPtr &in_classifications, aipe_msgs::ClassificationArray::Ptr &filtered_classifications);

        /*!
        * Publish the status of CAS overshooting according to apm_status, traffic light detection and latest traffic light position
        * @param detection_available true if any traffic light detected
        */
        void PublishOvershooting(const bool &detection_available);

        /*!
        * Publish all inputs with the output of the node
        * @param output result of the traffic light control
        */
        void PublishTrafficLightLog(const aipe_msgs::Classification &output);

        /*!
        * Publish traffic light signal 
        * @param in_msg input message to use the header
        * @param final_state final traffic light state
        * @param state_conf final state confidence (0 to 1)
        */
        bool PublishTLSignal(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg,
                             const std::string &final_state, const float &state_conf);

        /*!
        * Convert given final state and state confidence to a classification msg and publish using the given publisher
        * @param publisher publisher 
        * @param in_msg input message to use the header
        * @param final_state final traffic light state
        * @param state_conf final state confidence (0 to 1)
        */
        aipe_msgs::Classification PublishClassification(const ros::Publisher publisher, const std_msgs::Header &header,
                                                        const std::string &final_state, const float &state_conf);

    private:
        // Node handle and publisher
        ros::NodeHandle node_handle_;
        ros::Publisher publisher_junction_;
        ros::Publisher publisher_junction_marker_;
        ros::Publisher publisher_traffic_lights_;
        ros::Publisher publisher_tl_markers_;
        ros::Publisher publisher_precision_alignment_;
        ros::Publisher publisher_overshoot_;
        ros::Publisher publisher_tl_log_;
        ros::Publisher publisher_tl_cas_;
        ros::Publisher publisher_tl_junction_;

        // Subscribers
        ros::Subscriber detections_tl_subscriber_;
        ros::Subscriber classifications_fbc_subscriber_;
        ros::Subscriber classifications_fbl_subscriber_;
        ros::Subscriber classifications_fbr_subscriber_;
        ros::Subscriber speed_subscriber_;
        ros::Subscriber lane_subscriber_;
        // representing estimated movement of APM-> 0:moving straight, 1: turn right, 2:turn left, 3:based on apm behaviour
        ros::Subscriber light_indicator_subscriber_;
        ros::Subscriber hazard_subscriber_;
        ros::Subscriber trip_job_subscriber_;
        ros::Subscriber apm_status_subscriber_;
        ros::Subscriber cam_info_subscriber_;
        ros::Subscriber odom_subscriber_;
        ros::Subscriber junction_id_subscriber_;

        float apm_speed_;
        // lane index of APM received from the laneCallback (format:"0.0")
        std::string current_lane_;
        // light_indicator representing movement of APM (0: moving straight, 1:turn right, 2:turn left, 3:based on apm behaviour)
        int light_indicator_;
        // hazard situation representing availability of the road on right-turn. This flag is used to give right-turn signal while the straight is green and road is empty
        bool junction_hazard_;
        bool inside_bay_;
        // id representing which side of the APM is being used to traffic light control.
        std::string target_location_id_;
        // apm status
        int apm_status_;
        // max time (secs) allowed for CAS alignment
        int max_CAS_duration_;
        // Estimator for final state of the junction wrt localized classification results
        boost::shared_ptr<aipe::JunctionStateEstimator> junction_estimator_;
        // Classification results of front top centre camera filtered by CAS lights
        aipe_msgs::ClassificationArray::Ptr classifications_tbc_;
        // Classification results of front bottom left camera filtered by CAS lights
        aipe_msgs::ClassificationArray::Ptr classifications_tbl_;
        // Classification results of front bottom right camera filtered by CAS lights
        aipe_msgs::ClassificationArray::Ptr classifications_tbr_;
        // Classification results of front bottom right camera
        aipe_msgs::ClassificationArray::Ptr classifications_tbr_raw_;
        // message to log all inputs and output of the traffic light control node
        aipe_msgs::TrafficLightLog tl_log_;
        // detected objects
        aipe_msgs::DetectedObjectArray detected_objs_;
        // time of last classification results of front top centre received
        std::chrono::time_point<std::chrono::system_clock> tbc_time_;
        // time of last classification results of front bottom left received
        std::chrono::time_point<std::chrono::system_clock> tbl_time_;
        // time of last classification results of front bottom right received
        std::chrono::time_point<std::chrono::system_clock> tbr_time_;
        // list of traffic lights
        std::vector<std::string> CAS_color_list_;
        // CAS traffic light estimator to eliminate the outliers
        boost::shared_ptr<aipe::CASStateEstimator> CAS_estimator_;
        // threshold for red detection between cas_up/cas_down to decide whether it's solid light
        int precision_alignment_thresh_;
        // x coordinate of the latest detected traffic light. Used to determine overshooting scenario
        int CAS_tl_x_coordinate_;
        // camera image widths
        int left_cam_width_;
        int right_cam_width_;
        int top_cam_width_;
        // APM pose
        geometry_msgs::PoseWithCovariance apm_pose_;
        // Junction id (ppt09, ppt10, etc)
        std::string junction_id_;
    };
}

#endif //PROJECT_tl_vision_control_H
