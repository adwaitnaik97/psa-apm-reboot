#include "tl_vision_control/PSA_exceptional_cases.h"
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <aipe_utils/transformations.h>
#include <traffic_light_common/tl_utils.h>
#include <traffic_light_common/tl_types.h>

aipe::PSAExceptionalCases::PSAExceptionalCases(int turn_right_wait_time) : turn_wait_time_(turn_right_wait_time),
                                                                           num_greens_(0)
{
}

aipe::PSAExceptionalCases::~PSAExceptionalCases()
{
}

bool aipe::PSAExceptionalCases::CheckExceptionalCases(const int &light_indicator, const std::string &junction,
                                                      const std::string &current_lane, const bool &junction_hazard,
                                                      std::string &final_state, float &state_conf)
{
    bool status = true;

    // count consecutive greens to wait on right turn on straight green signal
    if (final_state == "Green")
    {
        ++num_greens_;
    }
    else
    {
        num_greens_ = 0;
    }

    if (light_indicator == 1 /*&& m_junction_hazard*/ && final_state == "Green")
    {
        // Temporarily check for CJ3 since one side of the junction is closed and having straight green and righht arrow
        // causes detection problem.
        /*if (junction == "CJ3" && current_lane == "8.2")
        {
            final_state = "right_arrow";
            state_conf = 1;
        }*/
        // allow right turn on straigt green if there is no vehicle in the junction
        if ((junction == "CJ1" && current_lane == "40.2") && !junction_hazard)
        {
            // wait to be sure that junction is empty. Give enough time to the vehicles waiting on the opposite lane to enter the junction
            // as this scenario is not handled by the tl_junction_hazard flag.
            if (num_greens_ > turn_wait_time_)
            {
                final_state = "right_arrow";
                state_conf = 1;
                //std::cout << "Green to right arrow, num greens: " << num_greens_ << std::endl;
            }
        }
        else
        {
            // invalid result
            status = false;
        }
    }
    else if(light_indicator == 0)
    {
        // TJ8 night time red right-arrow confusion
        if(current_lane == "55.2" && final_state == "cas_down")
        {
            final_state = "green";
            state_conf = 1;
        }
    }
    return status;
}
