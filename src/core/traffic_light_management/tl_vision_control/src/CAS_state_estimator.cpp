#include "tl_vision_control/CAS_state_estimator.h"
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <aipe_utils/transformations.h>
#include <traffic_light_common/tl_utils.h>
#include <traffic_light_common/tl_types.h>

aipe::CASStateEstimator::CASStateEstimator(const int &flickering_duration,
                                           const int &red_tolerance,
                                           const std::vector<std::string> &cas_color_list,
                                           const int &max_CAS_duration) : flickering_duration_(flickering_duration),
                                                                          red_tolerance_(red_tolerance),
                                                                          cas_up_age_(1000),
                                                                          cas_down_age_(1000),
                                                                          red_age_(1000),
                                                                          red_invisible_(1000),
                                                                          last_state_("-1"),
                                                                          max_CAS_duration_(max_CAS_duration),
                                                                          CAS_active_(false)
{
    CAS_color_list_ = cas_color_list;
}

aipe::CASStateEstimator::~CASStateEstimator()
{
}

bool aipe::CASStateEstimator::ResetValidation(const std::string &final_state)
{
    /*last_state_ = final_state;
    m_state_age = 0;
    m_old_age = 0;
    m_cand_state = "";*/
}

bool aipe::CASStateEstimator::ValidateState(std::string &final_state)
{
    //std::cout << "Initial state: " << final_state;
    // Do not publish new state on the first decision. Wait 3 consecutive estimation of the same state to give final decision
    bool validated = false;
    ++cas_up_age_;
    ++cas_down_age_;

    if (final_state.compare("Cas_up") == 0)
    {
        cas_up_age_ = 0;
    }
    else if (final_state.compare("Cas_down") == 0)
    {
        cas_down_age_ = 0;
    }

    if (final_state.compare("Red") == 0)
    {
        ++red_age_;
        red_invisible_ = 0;
    }
    else
    {
        red_age_ = 0;
        ++red_invisible_;
    }

    //std::cout << "final_state: " << final_state << std::endl;
    // Do not switch to Red or invalid state immediately wait until defined threshold time to prevent flickering
    if (final_state.compare("-1") == 0 || final_state.compare("") == 0)
    {
        // Store latest up or down signal until it expires
        if (cas_up_age_ < cas_down_age_ && cas_up_age_ <= flickering_duration_)
        {
            final_state = "Cas_up";
        }
        else
        {
            if (cas_down_age_ <= flickering_duration_)
            {
                final_state = "Cas_down";
            }
        }
    }
    // do not publish red signal for the first 'red_tolerance' frames. It's added to ignore red classifications on cas_up/cas_down flickering.
    else if (final_state.compare("Red") == 0)
    {
        if (red_age_ <= red_tolerance_)
        {
            final_state = last_state_;
        }
    }
    //std::cout << " validated state: " << final_state << std::endl;
    return validated;
}

bool aipe::CASStateEstimator::CASResult(const aipe_msgs::ClassificationArray::Ptr &classifications, std::string &result, int &tl_x_coordinate)
{
    bool found = false;
    std::unordered_map<std::string, int> status_counter;
    for (auto color : CAS_color_list_)
    {
        status_counter[color] = 0;
    }

    bool cas_up_found = false, cas_down_found = false;
    for (auto &obj : classifications->classifications)
    {
        // TODO: remove when classifier's confidence threshold works
        if (obj.confidence < 0.1)
        {
            continue;
        }

        if (status_counter.find(obj.class_name) != status_counter.end())
        {
            status_counter[obj.class_name]++;
            found = true;
        }

        if (obj.class_name.compare("Cas_up") == 0)
        {
            cas_up_found = true;
            tl_x_coordinate = obj.x + obj.width / 2;
        }
        if (obj.class_name.compare("Cas_down") == 0)
        {
            cas_down_found = true;
            tl_x_coordinate = obj.x + obj.width / 2;
        }
    }

    std::string second_best = "";
    std::string best_status = "";
    int best_count = 0;
    int second_best_count = 0;
    for (auto &count : status_counter)
    {
        if (count.second > best_count)
        {
            second_best = best_status;
            second_best_count = best_count;
            best_status = count.first;
            best_count = count.second;
        }
    }

    // Give priority to Red against Amber because sometimes cas_down is classified as red because of its location on the traffic light
    if (best_status.compare("Amber") == 0 && best_count == second_best_count && second_best.compare("Red") == 0)
    {
        best_status = "Red";
    }

    // if one of up or down is detected and the final result is not same, suppress the result
    // rely on counts if both or none of them is detected
    if (cas_up_found != cas_down_found)
    {
        best_status = (cas_down_found) ? "Cas_down" : "Cas_up";
    }

    result = best_status;
    return found;
}

bool aipe::CASStateEstimator::JunctionState(const aipe_msgs::DetectedObjectArray::ConstPtr &in_msg,
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
                                            bool &detection_available)
{
    // Publish classification result of top centre camera if available
    //if (!CASResult(classifications_tbc, final_state))
    {
        // Choose the related cameras according to the CAS target location
        std::string loc_id_str = target_location_id;
        boost::algorithm::to_lower(loc_id_str);
        if (loc_id_str.find("l1") != std::string::npos)
        {
            // front bottom right
            // Check if existing classification is valid
            auto cur_time = std::chrono::high_resolution_clock::now();
            auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - tbr_time).count();
            // Do not use the classification results older than 1 sec
            //if (time_diff < 1000)
            {
                CASResult(classifications_tbr, final_state, tl_x_coordinate);
                detection_available = classifications_tbr->classifications.size() != 0;
            }
        }
        else if (loc_id_str.find("l0") != std::string::npos)
        {
            //front bottom left
            // Check if existing classification is valid
            auto cur_time = std::chrono::high_resolution_clock::now();
            auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - tbl_time).count();
            // Do not use the classification results older than 1 sec
            //if (time_diff < 1000)
            {
                CASResult(classifications_tbl, final_state, tl_x_coordinate);
                detection_available = classifications_tbl->classifications.size() != 0;
            }
        }
    }

    // Classifier generally recognizes Cas down as red because of the location.
    // TODO: Temporarily modification until training reaches to the desired level.
    /*if (final_state.compare("Red") == 0)
    {
        final_state = "Cas_down";
    }*/

    // Red of CAS traffic light may be classified as Amber.
    if (final_state.compare("Amber") == 0)
    {
        final_state = "Red";
    }

    // Check state history to eliminate flickering scenario
    ValidateState(final_state);
    //std::cout << "Final State:      " << final_state << std::endl;
    last_state_ = final_state;
    red_age = red_age_;
    red_invisible = red_invisible_;

    // Force to publish red if max CAS duration exceed in the alignment job
    if (apm_status == 18) // alignment job
    {
        if (!CAS_active_)
        {
            // CAS traffic light recognition started
            if (final_state.compare("Red") == 0 || final_state.compare("Amber") == 0 || final_state.compare("Green") == 0 ||
                final_state.compare("Cas_down") == 0 || final_state.compare("Cas_up") == 0)
            {
                CAS_active_ = true;
                CAS_start_time_ = std::chrono::high_resolution_clock::now();
            }
        }
        auto cur_time = std::chrono::high_resolution_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(cur_time - CAS_start_time_).count();
        // If traffic light is active and reached max allowed duration
        if (CAS_active_ && time_diff > max_CAS_duration_ && !(final_state.compare("") == 0 || final_state.compare("-1") == 0))
        {
            final_state = "Red";
            ROS_INFO("[%s] Reached max alignment time(%d secs)! Red signal will be published after this, until the next job", __APP_NAME__, max_CAS_duration_);
        }
    }
    else
    {
        // reset the timer for CAS alignment
        CAS_active_ = false;
    }

    return true;
}