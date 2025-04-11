#include "tl_vision_control/junction_state_estimator.h"
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <aipe_utils/transformations.h>
#include <traffic_light_common/tl_utils.h>
#include <traffic_light_common/tl_types.h>
#include <tl_vision_control/side_tl_detector.h>
#include "tl_vision_control/PSA_exceptional_cases.h"

namespace atlc = aipe::traffic_light_common;

aipe::JunctionStateEstimator::JunctionStateEstimator(const std::string &lanes_file, const bool &virtual_poles,
                                                     const int &turn_right_wait_time, const int image_width) : last_state_(""),
                                                                                                               cand_state_(""),
                                                                                                               state_age_(0),
                                                                                                               old_age_(0),
                                                                                                               invalid_count_(0),
                                                                                                               num_validation_(3),
                                                                                                               evaluation_distance_(50),
                                                                                                               num_greens_(0),
                                                                                                               turn_wait_time_(turn_right_wait_time)
{
    lanes_file_ = lanes_file;
    virtual_poles_ = virtual_poles;
    num_real_poles_ = 4;

    color_list_.push_back("Green");
    color_list_.push_back("Red");
    color_list_.push_back("Amber");
    color_list_.push_back("right_arrow");
    color_list_.push_back("Cas_up");
    color_list_.push_back("Cas_down");

    LoadDictionaries();

    side_tl_detector_ = boost::shared_ptr<aipe::SideTLDetector>(new aipe::SideTLDetector(image_width));
    PSA_checker_ = boost::shared_ptr<aipe::PSAExceptionalCases>(new aipe::PSAExceptionalCases(turn_right_wait_time));
}

aipe::JunctionStateEstimator::~JunctionStateEstimator()
{
}

bool aipe::JunctionStateEstimator::LoadDictionaries()
{
    direction_map_["straight"] = 0;
    direction_map_["right"] = 1;
    direction_map_["left"] = 2;

    try
    {
        // Load lanes
        std::ifstream lanes_file(lanes_file_);
        if (lanes_file.good())
        {
            boost::property_tree::ptree pt;
            boost::property_tree::read_json(lanes_file, pt);

            using boost::property_tree::ptree;
            // iterate junctions
            for (ptree::const_iterator it = pt.begin(); it != pt.end(); ++it)
            {
                std::string junction_name = it->first;
                // iterate lanes
                for (ptree::const_iterator it_lane = it->second.begin(); it_lane != it->second.end(); ++it_lane)
                {
                    std::string lane_id = it_lane->first;
                    std::string map_key = junction_name + '_' + lane_id;
                    atlc::Lane lane(lane_id);
                    // iterate directions
                    for (ptree::const_iterator it_direction = it_lane->second.begin(); it_direction != it_lane->second.end(); ++it_direction)
                    {
                        std::string direction = it_direction->first;
                        // iterate traffic light ids
                        for (ptree::const_iterator it_tls = it_direction->second.begin(); it_tls != it_direction->second.end(); ++it_tls)
                        {
                            std::string tl_id_str = it_tls->second.get_value<std::string>();
                            int tl_id = std::atoi(tl_id_str.c_str());
                            // Add virtual one with same info for each traffic light. Used to solve false detection/classification issue
                            if (virtual_poles_)
                            {
                                lane.m_tls[direction_map_[direction]].push_back((tl_id * 2) - 1);
                                lane.m_tls[direction_map_[direction]].push_back((tl_id * 2));
                            }
                            else
                            {
                                lane.m_tls[direction_map_[direction]].push_back(tl_id);
                            }
                        }
                    }
                    junction_lane_map_[map_key] = lane;
                }
            }
            ROS_INFO("[%s] Lane rules loaded successfully!", __APP_NAME__);
        }
        else
        {
            ROS_ERROR("[%s] Could not load lanes file: %s!", __APP_NAME__, lanes_file_.c_str());
        }
    }
    catch (std::exception const &e)
    {
        ROS_ERROR("[%s] Error while loading lanes file!", __APP_NAME__);
        std::cerr << e.what() << std::endl;
    }
    return true;
}

bool aipe::JunctionStateEstimator::JunctionConfidence(const aipe_msgs::DetectedObjectArray::Ptr &traffic_lights, const int &max_status, const std::unordered_map<int, int> &real_poles_map, float &state_conf)
{
    // half of the confidence represents the classification consistency of the poles
    float class_conf = std::min(((float)(max_status) / num_real_poles_) * 0.5, 0.5);
    class_conf = std::max(class_conf, 0.0f);
    int num_detects = 0;
    int num_poles = traffic_lights->objects.size();
    if (virtual_poles_)
        num_poles /= 2;
    for (int i = 0; i < num_poles; ++i)
    {
        if (real_poles_map.find(i + 1) != real_poles_map.end())
        {
            ++num_detects;
        }
    }
    // half of the confidence represents the visibility of the poles for the current lane
    float visibility_conf = std::min(((float)num_detects / (float)num_real_poles_) * 0.5f, 0.5f);
    visibility_conf = std::max(visibility_conf, 0.0f);
    state_conf = class_conf + visibility_conf;
    return true;
}

bool aipe::JunctionStateEstimator::ResetValidation(const std::string &final_state)
{
    last_state_ = final_state;
    state_age_ = 0;
    old_age_ = 0;
    cand_state_ = "";
}

bool aipe::JunctionStateEstimator::ValidateState(std::string &final_state)
{
    // Do not publish new state on the first decision. Wait 3 consecutive estimation of the same state to give final decision
    bool validated = false;
    // If there is no candidate state waiting to be validated
    if (cand_state_ == "")
    {
        // start validating a new candidate
        if (final_state != last_state_)
        {
            old_age_ = state_age_;
            state_age_ = 0;
            cand_state_ = final_state;
            final_state = last_state_;
        }
        else
        {
            ++state_age_;
        }
    }
    else // there is an active candidate state waiting for validation
    {
        if (final_state == cand_state_)
        {
            ++state_age_;
            // if candidate is validated by 3 consecutive estimation, set as current state
            if (state_age_ >= num_validation_)
            {
                final_state = cand_state_;
                last_state_ = final_state;
                cand_state_ = "";
                validated = true;
            }
            else
            {
                // still waiting for validation, use previous state
                if (final_state != last_state_)
                {
                    final_state = last_state_;
                }
            }
        }
        else
        {
            // ignore the single false detection and return back to previous state
            if (final_state == last_state_)
            {
                cand_state_ = "";
                state_age_ = old_age_ + 1;
            }
            else // new state different than previous and candidate
            {
                old_age_ = state_age_;
                state_age_ = 0;
                cand_state_ = final_state;
                final_state = last_state_;
            }
        }
    }
    return validated;
}

bool aipe::JunctionStateEstimator::JunctionState(const aipe_msgs::DetectedObjectArray::Ptr &traffic_lights, const aipe_msgs::ClassificationArray::Ptr &br_classifications, const std::string &junction,
                                                 const int &light_indicator, const std::string &current_lane,
                                                 const bool &junction_hazard, std::string &final_state, float &state_conf)
{
    // Validate required information for junction state estimation
    if (traffic_lights->objects.empty())
    {
        //ROS_INFO("[%s] Junction tl list is empty!", __APP_NAME__);
        return false;
    }
    std::unordered_map<std::string, int> status_counter;
    for (auto color : color_list_)
    {
        status_counter[color] = 0;
    }

    // do not evaluate traffic lights if they are more than 50 meters away
    bool eval_active = false;
    bool any_color = false;

    std::vector<aipe_msgs::DetectedObject> filtered_tls;
    for (auto &tl : traffic_lights->objects)
    {
        filtered_tls.push_back(tl);
    }
    // filter out side traffic lights when APM moves straight. Do not filter out at TJ8 lane 55.2 which has red right arrows
    if (light_indicator == 0 && current_lane != "55.2")
    {
        // ignore side traffic lights
        side_tl_detector_->FilterSideTrafficLights(filtered_tls);
    }

    // Count colors of current traffic lights
    for (const auto &tl : filtered_tls)
    {
        bool found = false;
        // No need to look TL status if it's right traffic light
        // This control was inside the tls comparison below but sometimes traffic lights may be matched with wrong poles
        // that causes to miss the classified TL.
        if (tl.label.compare("Right_traffic_light") == 0)
        {
            status_counter["right_arrow"]++;
            any_color = true;
            found = true;
            // Need to change tl status to right_arrow?
        }
        else
        {
            std::string color = aipe::TLUtils::GetTLStatus(tl);
            // replace Amber with Red to stop at Amber
            if (color == "Amber")
            {
                color = "Red";
            }
            // TJ8 exception that occurs at night because of red right-arrow
            /*if (current_lane == "55.2" && aipe::TLUtils::IsNight() && color == "Cas_down")
            {
                color = "Green";
            }*/
            if (status_counter.find(color) != status_counter.end())
            {
                any_color = true;
                status_counter[color]++;
                found = true;
            }
        }
    }

    // TODO: move to exceptional cases class after merge
    // Check bottom right camera's clasification results to get if there is a green light detection on lane 55.2
    if (current_lane == "55.2" && junction == "TJ8")
    {
        for (auto br_class : br_classifications->classifications)
        {
            if (br_class.class_name == "Green")
            {
                status_counter["Green"]++;
                break;
            }
        }
    }

    bool invalid_return = false;
    if (!any_color)
    {
        //ROS_INFO("[%s] No traffic light status available !", __APP_NAME__);
        final_state = last_state_;
        //Count no detections to re-initialize the status
        ++invalid_count_;
        invalid_return = true;
    }

    // Do not generate state if junction is far than the distance threshold
    /*if (!eval_active)
    {
        ++invalid_count_;
        invalid_return = true;
    }*/

    if (invalid_return)
    {
        if (invalid_count_ >= num_validation_)
        {
            //ROS_INFO("[%s] Invalid because of localization miss", __APP_NAME__);
            ResetValidation("");
        }
        return true;
    }

    invalid_count_ = 0;

    // find the status with maximum hits to eliminate the outliers
    std::pair<std::string, int> best_status;
    std::pair<std::string, int> second_status;
    for (auto color_count : status_counter)
    {
        if (color_count.second >= best_status.second)
        {
            second_status = best_status;
            best_status = color_count;
        }
        else
        {
            if (color_count.second > second_status.second)
            {
                second_status = color_count;
            }
        }
    }

    final_state = best_status.first;
    // Use latest state if current detections are inconsistent
    if (best_status.second == second_status.second && (light_indicator != 1 && second_status.first.compare("right_arrow") != 0))
    {
        std::string best_str = best_status.first;
        std::string second_str = second_status.first;
        ROS_INFO("[%s] Inconsistent TLs detected: %s and %s at the same time", __APP_NAME__, best_str.c_str(), second_str.c_str());
        final_state = last_state_;
    }

    // Calculate confidence of junction state
    //JunctionConfidence(traffic_lights, best_status.second, real_poles_map, state_conf);
    state_conf = 1;

    bool skip_validation = false;

    // If APM is going to turn right and there are right arrows classified currently, suppress the other status
    if (light_indicator == 1 && (status_counter["right_arrow"] > 0 /*|| (best_status.first == "Green" && !m_junction_hazard)*/))
    {
        final_state = "right_arrow";
        //last_state_ = final_state;
        //state_conf = (float)(status_counter["right_arrow"]) / (float)tls.size();
        state_conf = 1;
    }
    //std::cout << "junction hazard: " << junction_hazard << " final state: " << final_state << " light indicator: " << light_indicator << std::endl;
    //std::cout << "num greens: " << num_greens_ << std::endl;

    bool valid_result = PSA_checker_->CheckExceptionalCases(light_indicator, junction, current_lane, junction_hazard, final_state, state_conf);
    if (!valid_result)
    {
        final_state = "Red";
        skip_validation = true;
        ResetValidation(final_state);
    }

    // Do not generate right_arrow if APM is going straight
    if (light_indicator == 0 && final_state == "right_arrow")
    {
        // generate second most occuring state if any exist, otherwise generate the last state again
        if (second_status.second > 0)
            final_state = second_status.first;
        else
            final_state = last_state_;
    }

    // Convert amber to red, required to stop when TL turns red on the stop line.
    if (final_state == "Amber")
    {
        final_state = "Red";
    }

    // Validate current state by comparing the previous states to eliminate the outliers.
    if (!skip_validation)
        bool validated = ValidateState(final_state);

    //last_state_ = final_state;

    return true;
}
