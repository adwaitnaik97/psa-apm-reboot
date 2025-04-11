#ifndef PSA_EXCEPTIONAL_CASES_H
#define PSA_EXCEPTIONAL_CASES_H

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
    * This class defines the exceptional cases specified for PSA APM project. All project-specific checks are done by this class. 
    * TODO: These controls should be moved to situation cognition.
    */
    class PSAExceptionalCases
    {
    public:
        /*!
        * @param turn_wait_time               wait time (in frames) before generating right_arrow on straight green (for predefined junctions) 
        *                                     to prevent possiblity of false detection
        */
        PSAExceptionalCases(int turn_wait_time);
        ~PSAExceptionalCases();

        /*!
        * Check PSA APM specific exceptional cases and modify the final state if required
        * @param light_indicator            light indicator representing apm direction (0: straight, 1: right turn, 2:left turn)
        * @param junction                   current junction name (i.e. CJ1)
        * @param current_lane               current lane_id
        * @param junction_hazard            boolean flag representing the junction hazard status
        * @param final_state                state of the junction ("red", "amber", "green", "right_arrow")
        * @param state_conf                 final state confidence to be updated
        * @return status                    true if a valid state is assigned, false otherwise. Returned value must be processed to handle invalid states
        */
        bool CheckExceptionalCases(const int &light_indicator, const std::string &junction,
                                   const std::string &current_lane, const bool &junction_hazard,
                                   std::string &final_state, float &state_conf);

    private:
        // number of consecutive green signals. Used to wait for a while to be sure it's safe to turn right on straight green.
        int num_greens_;
        // wait time (number of frames) in case of right turn on straight green
        int turn_wait_time_;
    };
}

#endif //PSA_EXCEPTIONAL_CASES_H
