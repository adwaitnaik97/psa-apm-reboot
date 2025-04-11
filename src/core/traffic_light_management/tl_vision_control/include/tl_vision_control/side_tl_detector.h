#ifndef SIDE_TL_DETECTOR_H
#define SIDE_TL_DETECTOR_H

#define __APP_NAME__ "side_tl_detector"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include "aipe_msgs/DetectedObject.h"
#include "aipe_msgs/ClassificationArray.h"
#include "airs_msgs/reg37.h"
#include "traffic_light_common/tl_types.h"
#include "json/json.h"

namespace aipe
{
    /*!
    * Side traffic light detector by clustering traffic lights. Traffic lights that are on the same pole and have different signals (red-green)
    * determined as straight and side ones according to their position on the image. i.e. if traffic light pole is on the left side of the lane,
    * the traffic light located on left on this pole belongs to the current lane while the right one is looking for the side lane.
    */
    class SideTLDetector
    {
    public:
        SideTLDetector(int image_width);
        ~SideTLDetector();
        /*!
        * Analyze traffic lights according to their position and neighbourhood to determine the side traffic lights. Filter out the sides.
        * @param tls      detected traffic light, the ones determined as side traffic light will be removed.
        * @return status true if succesfully filtered out, false in case of error
        */
        bool FilterSideTrafficLights(std::vector<aipe_msgs::DetectedObject> &tls);

        /*!
        * Compare given two traffic light detections using their x coordinate on the image. If the distance between the closest 
        * edges are smaller than a traffic light's width, then they are assumed as close enough
        * @param tl1      traffic light object to be compared
        * @param tl2      traffic light object to be compared
        * @return true if they are close enogh, false otherwise
        */
        bool CloseEnough(const aipe_msgs::DetectedObject &tl1, const aipe_msgs::DetectedObject &tl2);

        /*!
        * Group given traffic lights according to the their positions. Traffic lights belonging to the same pole are grouped together 
        * @param tls            list of traffic lights
        * @param tl_groups      dictionary containing list of grouped traffic lights for each pole
        * @return true if succesfully grouped, false in case of an error
        */
        bool GroupPoles(const std::vector<aipe_msgs::DetectedObject> &tls, std::unordered_map<int, std::vector<aipe_msgs::DetectedObject>> &tl_groups);

        /*!
        * Filter out the side traffic lights and get only the straight ones with the dictionary
        * @param tl_groups      dictionary containing list of grouped straight traffic lights
        * @return true if succesfully filtered, false otherwise
        */
        bool GetStraightTLs(std::unordered_map<int, std::vector<aipe_msgs::DetectedObject>> &tl_groups);

        /*!
        * Remove the objects whose indices are provided.
        * @param pole_tls      list of the objects
        * @param indices       list of the indices to be removed from the pole_tls vector
        * @return true if succesfully removed, false otherwise
        */
        bool RemoveSideTLs(std::vector<aipe_msgs::DetectedObject> &pole_tls, std::vector<int> &indices);

        /*!
        * Compare the given two traffic lights to determine which is the side one
        * @param tl_groups     list of traffic light objects
        * @param index1        index of the traffic light object to be compared
        * @param index2        index of the traffic light object to be compared
        * @return index of the side traffic light, -1 if failed to find
        */
        int FindSideTL(const std::vector<aipe_msgs::DetectedObject> &tl_groups, int index1, int index2);

    private:
        int image_width_;
     };
}

#endif //SIDE_TL_DETECTOR_H
