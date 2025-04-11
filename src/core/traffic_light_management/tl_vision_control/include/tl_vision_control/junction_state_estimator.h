#ifndef JUNCTION_STATE_ESTIMATOR_H
#define JUNCTION_STATE_ESTIMATOR_H

#define __APP_NAME__ "tl_vision_control"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include "aipe_msgs/DetectedObjectArray.h"
#include "aipe_msgs/ClassificationArray.h"
#include "airs_msgs/reg37.h"
#include "traffic_light_common/tl_types.h"
#include "json/json.h"

namespace aipe
{
    /*!
    * Traffic light junction state estimator
    */
    class SideTLDetector;
    class PSAExceptionalCases;
    class JunctionStateEstimator
    {
    public:
        JunctionStateEstimator(const std::string &lanes_file, const bool &virtual_poles,
                               const int &turn_right_wait_time, const int image_width);
        ~JunctionStateEstimator();

        /*!
        * Load lane-traffic light information provided by the config files.
        * @param status true if files loaded successfully, false otherwise
        */
        bool LoadDictionaries();

        /*!
        * Calculate junction state confidence considering consistency of traffic lights' status and visibility of traffic lights exist 
        * in the current rule list of lane and light indicator.
        * @param traffic_lights            list of traffic lights belonging to the current junction
        * @param max_status                number of staus (green, red, amber, etc.) which is most classified in the junction
        * @param real_poles_map            map containing detection status of each real pole in the junction. 
        * @param state_conf                state confidence between 0.0 and 1.0
        * @return status true if state calculated, false otherwise.
        */
        bool JunctionConfidence(const aipe_msgs::DetectedObjectArray::Ptr &traffic_lights, const int &max_status,
                                const std::unordered_map<int, int> &real_poles_map, float &state_conf);

        /*!
        * Reset all validation parameters
        * @param final_state               state of the junction ("red", "amber", "green", "right_arrow")
        * @return status 
        */
        bool ResetValidation(const std::string &final_state);

        /*!
        * Calculate junction state according to APM lane, direction and status of traffic lights of the junction.
        * @param traffic_lights            list of traffic lights belonging to the current junction
        * @param br_classifications           classification results of bottom right camera. Added temporary to solve lane 55.2 issue on PSA. (TODO: move to exceptional cases)
        * @param junction                  junction name (i.e. CJ2)
        * @param light_indicator           flag representing APM's planned direction (0: Straight, 1: Right, 2:Left)
        * @param lane_id                   current lane id received from aidc
        * @param junction_hazard           junction hazard status (true/false)
        * @param final_state               state of the junction ("red", "amber", "green", "right_arrow")
        * @param state_conf                state confidence between 0.0 and 1.0
        * @return status true if state calculated, false otherwise.
        */
        bool JunctionState(const aipe_msgs::DetectedObjectArray::Ptr &traffic_lights, const aipe_msgs::ClassificationArray::Ptr &br_classifications, const std::string &junction, const int &light_indicator,
                           const std::string &current_lane, const bool &junction_hazard, std::string &final_state, float &state_conf);

        /*!
        * Compare final state with the previous states to eliminate false detections
        * @param final_state               state of the junction ("red", "amber", "green", "right_arrow")
        * @return status true if state validated, false otherwise.
        */
        bool ValidateState(std::string &final_state);

    private:
        // store valid traffic lights for each lane with "junction_laneId" key in the map
        std::unordered_map<std::string, aipe::traffic_light_common::Lane> junction_lane_map_;
        // store plain string and ApmMove pair of moving directions ("straight,0")
        std::unordered_map<std::string, int> direction_map_;
        // list of traffic lights
        std::vector<std::string> color_list_;
        // flag to create a virtual pole at the same utm coordinate for each given pole. It's used to improve the accuracy of junction state
        bool virtual_poles_;
        // number of real poles
        int num_real_poles_;
        // number of consecutive decisions
        int state_age_;
        int old_age_;
        // number of consecutive estimation for validation
        int num_validation_;
        // number of consecutive invalid state
        int invalid_count_;
        // last state of the junction
        std::string last_state_;
        // candidate state waiting for validation (3 consecutive frames)
        std::string cand_state_;
        // maximum distance to TL for evaluation
        int evaluation_distance_;
        // json file storing lane-traffic light relation
        std::string lanes_file_;
        // number of consecutive green signals. Used to wait for a while to be sure it's safe to turn right on straight green.
        int num_greens_;
        // wait time (number of frames) in case of right turn on straight green
        int turn_wait_time_;
        // class to determine if the traffic light looking straight or side
        boost::shared_ptr<aipe::SideTLDetector> side_tl_detector_;
        // Class to check exceptional cases specified for PSA APM
        boost::shared_ptr<aipe::PSAExceptionalCases> PSA_checker_;
    };
}

#endif //JUNCTION_STATE_ESTIMATOR_H
