#ifndef CAS_STATE_ESTIMATOR_H
#define CAS_STATE_ESTIMATOR_H

#define __APP_NAME__ "tl_vision_control"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include "aipe_msgs/DetectedObjectArray.h"
#include "aipe_msgs/ClassificationArray.h"
#include "airs_msgs/reg37.h"
#include "traffic_light_common/tl_types.h"
#include "json/json.h"
#include <chrono>

namespace aipe
{
    /*!
    * Traffic light CAS state estimator
    */
    class CASStateEstimator
    {
    public:
        /*!
        * @param max_flickering_duration Time distance threshold for cas_up and cas_down traffic lights. If any invalid signal is 
        * detected within this time period, it's ignored by assuming a blinking traffic light scenario.
        * @param red_tolerance Less than this number of consecutive red detections are ignored. Latest status is kept publishing during this period 
        * @param cas_color_list List of CAS traffic lights. Any classification results which is not listed here is not published 
        * @param max_CAS_duration Maximum number of seconds allowed for a CAS alignment 
        */
        CASStateEstimator(const int &flickering_duration, const int &red_tolerance,
                          const std::vector<std::string> &cas_color_list, const int &max_CAS_duration);
        ~CASStateEstimator();

        /*!
        * Reset all validation parameters
        * @param final_state               state of the junction ("red", "amber", "green", "right_arrow")
        * @return status 
        */
        bool ResetValidation(const std::string &final_state);

        /*!
        * Compare final CAS TL state with the previous states to eliminate flickering scenario. This function is used for bay area.
        * Check juntion_state_estimator for junction traffic lights
        * @param final_state               state of the junction ("red", "cas_up", "cas_down")
        * @return status true if state validated, false otherwise.
        */
        bool ValidateState(std::string &final_state);

        /*!
        * Estimates CAS traffic light state considering APM location and corresponding camera (bottom left/right) to detect/classify TL.
        * @param in_msg                    input objects
        * @param target_location_id        target traffic light location id deciding which side camera is going to be used. 'L1' or 'l1' for 
        *                                  bottom right camera.
        * @param classifications_tbc       classification results of front top centre camera
        * @param classifications_tbr       classification results of front bottom right camera
        * @param tbr_time                  timestamp of latest tbr classification data. Used to ignore expired classifications
        * @param classifications_tbl       classification results of front bottom left camera
        * @param tbl_time                  timestamp of latest tbr classification data. Used to ignore expired classifications
        * @param final_state               final state to be estimated
        * @param apm_status                apm status representing the assigned job
        * @param red_age                   number of consecutive red detections (red detections might be supressed)
        * @param red_invisible             number of consecutive frames that red light is not detected 
        * @param tl_x_coordinate           x coordinate of the detected traffic light
        * @param detection_available       true if any traffic light detection available
        * @return status true if state estimated, false otherwise.
        */
        bool JunctionState(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg,
                           const std::string &target_location_id,
                           const aipe_msgs::ClassificationArray::Ptr &classifications_tbc,
                           const aipe_msgs::ClassificationArray::Ptr &classifications_tbr,
                           const std::chrono::time_point<std::chrono::system_clock> &tbr_time,
                           const aipe_msgs::ClassificationArray::Ptr &classifications_tbl,
                           const std::chrono::time_point<std::chrono::system_clock> &tbl_time,
                           const int &apm_status,
                           std::string &final_state,
                           int &red_age,
                           int &red_invisible,
                           int &tl_x_coordinate,
                           bool &detection_available);

        /*!
        * Extract the result traffic light status from the given classification results
        * @param classifications classification results
        * @param result result status to be filled if available
        * @param tl_x_coordinate x coordinate of the traffic light. 
        * @return true if valid result exists, false otherwise
        */
        bool CASResult(const aipe_msgs::ClassificationArray::Ptr &classifications, std::string &result, int &tl_x_coordinate);

    private:
        // max allowed duration of a flickering period. Invisibility of cas_up or cas_down less than this period is tolerated.
        int flickering_duration_;
        // Less than this number of consecutive red detections are ignored. Latest status is kept publishing during this period.
        int red_tolerance_;
        // list of traffic lights
        std::vector<std::string> CAS_color_list_;
        // age of last cas_up signal
        int cas_up_age_;
        // age of last cas_down signal
        int cas_down_age_;
        // age of the last red signal
        int red_age_;
        // numer of consecutive frames that red is invisible 
        int red_invisible_;
        // last state
        std::string last_state_;
        // CAS traffic light recognition is disabled after a predefined time (50 secs) for safety concerns.
        int max_CAS_duration_;
        // Time of first CAS traffic light signal
        std::chrono::time_point<std::chrono::system_clock> CAS_start_time_;
        // flag indicates that CAS traffic light is active
        bool CAS_active_;
    };
}

#endif //CAS_STATE_ESTIMATOR_H
