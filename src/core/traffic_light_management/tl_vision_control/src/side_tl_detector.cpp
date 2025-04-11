#include "tl_vision_control/side_tl_detector.h"
#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <aipe_utils/transformations.h>
#include <traffic_light_common/tl_utils.h>
#include <traffic_light_common/tl_types.h>

aipe::SideTLDetector::SideTLDetector(int image_width) : image_width_(image_width)
{
}

aipe::SideTLDetector::~SideTLDetector()
{
}

bool aipe::SideTLDetector::GroupPoles(const std::vector<aipe_msgs::DetectedObject> &tls, std::unordered_map<int, std::vector<aipe_msgs::DetectedObject>> &tl_groups)
{
    bool status = true;
    size_t tl_size = tls.size();

    // iterate detected traffic lights
    for (int i = 0; i < tl_size; ++i)
    {
        // compare the current tl with the ones already assigned to a pole.
        bool matched = false;
        for (std::unordered_map<int, std::vector<aipe_msgs::DetectedObject>>::const_iterator it = tl_groups.begin(); it != tl_groups.end(); it++)
        {
            size_t pole_size = (*it).second.size();
            for (int p = 0; p < pole_size; ++p)
            {
                // add to the pole if it's close enough according to the horizontal position on the image
                if (CloseEnough(tls[i], (*it).second[p]))
                {
                    tl_groups[(*it).first].push_back(tls[i]);
                    matched = true;
                    break; // TODO: may need to find closest one if there are multiple in case of two traffic light poles look close because of the perspective
                }
            }
            // do not continue to find a better match for now since poles must be distant enough
            if (matched)
            {
                break;
            }
        }
        // create a new pole if there is no matched
        if (!matched)
        {
            int new_group_id = tl_groups.size();
            tl_groups[new_group_id] = std::vector<aipe_msgs::DetectedObject>();
            tl_groups[new_group_id].push_back(tls[i]);
        }
    }
    return status;
}

bool aipe::SideTLDetector::CloseEnough(const aipe_msgs::DetectedObject &tl1, const aipe_msgs::DetectedObject &tl2)
{
    // tl1 is on the right
    if (tl1.x > tl2.x)
    {
        // assume they are close enough if the distance between the close edges are smaller than a traffic light width
        if (tl2.x + tl2.width > tl1.x - tl1.width)
        {
            return true;
        }
    }
    else
    {
        if (tl1.x + tl1.width > tl2.x - tl2.width)
        {
            return true;
        }
    }
    return false;
}

int aipe::SideTLDetector::FindSideTL(const std::vector<aipe_msgs::DetectedObject> &tls, int index1, int index2)
{
    int index = -1;
    aipe_msgs::DetectedObject tl1 = tls[index1];
    aipe_msgs::DetectedObject tl2 = tls[index2];

    // remove either green or red on the pole according to the pole position and tl position on the pole

    // remove right tl if the pole is on the left side of the lane
    if (tl1.x < image_width_ / 2)
    {
        if (tl1.x < tl2.x)
        {
            return index2;
        }
        else
        {
            return index1;
        }
    }
    else // remove left one if the pole is on the right side of the lane
    {
        if (tl1.x < tl2.x)
        {
            return index1;
        }
        else
        {
            return index2;
        }
    }

    return index;
}

bool aipe::SideTLDetector::RemoveSideTLs(std::vector<aipe_msgs::DetectedObject> &pole_tls, std::vector<int> &indices)
{
    bool status = true;
    try
    {
        // sort the indices to start removing from the end
        std::sort(indices.begin(), indices.end(), std::greater<int>());
        for (auto i : indices)
        {
            pole_tls.erase(pole_tls.begin() + i);
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[%s] %s", __APP_NAME__, e.what());
        return false;
    }
    return status;
}

bool aipe::SideTLDetector::GetStraightTLs(std::unordered_map<int, std::vector<aipe_msgs::DetectedObject>> &tl_groups)
{
    bool status = false;
    // iterate tl poles
    for (std::unordered_map<int, std::vector<aipe_msgs::DetectedObject>>::iterator it = tl_groups.begin(); it != tl_groups.end(); it++)
    {
        // no need to analyze if there is only one detected traffic light
        if((*it).second.size() == 1)
        {
            continue;
        }
        bool green_found = false, red_found = false;
        int green_index = -1, red_index = -1;
        std::vector<int> sides_to_be_removed;
        // iterate the tls on the same pole
        for (int i = 0; i < (*it).second.size(); ++i)
        {
            std::string color = aipe::TLUtils::GetTLStatus((*it).second[i]);
            int index_to_remove = -1;
            if (color == "Green")
            {
                // if current tl is green and there is a red detected on the same pole, find which one is side tl and add it to objects to be removed
                if (red_found)
                {
                    index_to_remove = FindSideTL((*it).second, i, red_index);
                    // set red_found flag false as it will be deleted.
                    if (index_to_remove == red_index)
                    {
                        red_found = false;
                    }
                }
                // update flags if the current one will not be deleted
                if (index_to_remove != i)
                {
                    green_found = true;
                    green_index = i;
                }
            }
            else if (color == "Red")
            {
                if (green_found)
                {
                    index_to_remove = FindSideTL((*it).second, i, green_index);
                    // set green flag false as it will be deleted.
                    if (index_to_remove == green_index)
                    {
                        green_found = false;
                    }
                }
                // update flags if the current one will not be deleted
                if (index_to_remove != i)
                {
                    red_found = true;
                    red_index = i;
                }
            }
            if (index_to_remove != -1)
            {
                sides_to_be_removed.push_back(index_to_remove);
            }
        }
        if (!sides_to_be_removed.empty())
        {
            RemoveSideTLs((*it).second, sides_to_be_removed);
            // return true if there is a change on the tl list, otherwise input tl list will be used as it is
            status = true;
        }
    }
    return status;
}

bool aipe::SideTLDetector::FilterSideTrafficLights(std::vector<aipe_msgs::DetectedObject> &tls)
{
    bool status = true;
    // store all traffic lights with belonging pole id in a dictionary
    // pole id, list of tls
    std::unordered_map<int, std::vector<aipe_msgs::DetectedObject>> tl_groups;

    // group the traffic lights belong to the same pole
    GroupPoles(tls, tl_groups);

    // if there are both red and green on the same pole, check the pole position to determine if pole is located on the left or right of the truck
    if (GetStraightTLs(tl_groups))
    {
        // update tls vector if there is a removed (side) traffic light
        tls.clear();
        for (auto it = tl_groups.begin(); it != tl_groups.end(); it++)
        {
            tls.insert(tls.end(), (*it).second.begin(), (*it).second.end());
        }
    }
    return false;
}