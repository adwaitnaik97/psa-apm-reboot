/* aide_map -> map
"rotationW":0.9204036661218294,
"rotationX":0.0,
"rotationY":0.0,
"rotationZ":0.39096942513385363,
"translationX":349.0048232448462
"translationY" :22.723749502154533
"translationZ": 0.0"
*/

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <nlohmann/json.hpp>

bool found = false;
double translationX;
double translationY;
double translationZ;
double qx;
double qy;
double qz;
double qw;
geometry_msgs::TransformStamped map_to_odom;


double vehTranslationX;
double vehTranslationY;
double vehYaw;


std::mutex mutex_pos;

geometry_msgs::TransformStamped createTransformStamped(double x, double y, double z, double qx,
                                                       double qy, double qz, double qw,
                                                       const std::string& parent_frame,
                                                       const std::string& child_frame) {
  geometry_msgs::TransformStamped transformStamped;

  // Set the frame IDs
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;

  // Set the translation
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;

  // Set the rotation (quaternion)
  transformStamped.transform.rotation.x = qx;
  transformStamped.transform.rotation.y = qy;
  transformStamped.transform.rotation.z = qz;
  transformStamped.transform.rotation.w = qw;

  // Optionally, set the timestamp
  transformStamped.header.stamp = ros::Time::now();

  return transformStamped;
};

void cb_func(const std_msgs::String msg) {
  std::string data = msg.data;
  nlohmann::json json_msg = nlohmann::json::parse(data);
  
  // 0 map
  // 2 odom
  // 1 base_link
  // if (json_msg["msgType"] == "tf") {
  //   if (json_msg["frameId"] == "0" && json_msg["childFrameId"] == "2") {
  //     found = true;

  //     translationX = json_msg["translationX"];
  //     translationY = json_msg["translationY"];
  //     translationZ = json_msg["translationZ"];
  //     qx = json_msg["rotationX"];
  //     qy = json_msg["rotationY"];
  //     qz = json_msg["rotationZ"];
  //     qw = json_msg["rotationW"];
  //   }
  // }

   if (json_msg["msgType"] == "ego_state") 
   {     
     vehTranslationX = json_msg["vehPositionX"];
     vehTranslationY = json_msg["vehPositionY"];
     vehYaw = json_msg["vehYaw"];
   }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vehicle_pose_in_map");
  ros::NodeHandle nh;

  // TF2 Buffer and Listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Subscriber sub = nh.subscribe("/aios/remote_handler/mqtt/to_client", 20, cb_func);
  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("/tf_point", 10);
  ros::Publisher point_pub2 = nh.advertise<geometry_msgs::PointStamped>("/ego_point", 10);
  std::string map_frame, odom_frame, base_link_frame;
  nh.param<std::string>("/remote_handler/aid_map_frame", map_frame, "");
  nh.param<std::string>("/remote_handler/aid_odom_frame", odom_frame, "");
  nh.param<std::string>("/remote_handler/aid_base_link_frame", base_link_frame, "");
 

  ros::Rate rate(20.0);  
  tf::TransformBroadcaster br;
  /*****/
  while (ros::ok()) {
    try {
      if (found) {
        // map_to_odom = createTransformStamped(translationX, translationY, translationZ, qx, qy, qz,
        //                                      qw, "aide_map", "map");

        // // Broadcast the transform
        // br.sendTransform(map_to_odom);

        geometry_msgs::TransformStamped map_to_odom =
            tfBuffer.lookupTransform(map_frame,  odom_frame, ros::Time(0), ros::Duration(1.0));

        geometry_msgs::TransformStamped odom_to_base_link =
            tfBuffer.lookupTransform(odom_frame, base_link_frame, ros::Time(0), ros::Duration(1.0));

        // Combine transforms: map -> base_link = (map -> odom) * (odom -> base_link)
        tf2::Transform tf_map_to_odom, tf_odom_to_base_link, tf_map_to_base_link;
        tf2::fromMsg(map_to_odom.transform, tf_map_to_odom);
        tf2::fromMsg(odom_to_base_link.transform, tf_odom_to_base_link);

        tf_map_to_base_link = tf_map_to_odom * tf_odom_to_base_link;

        // Extract position and orientation from the combined transform
        geometry_msgs::Transform final_transform = tf2::toMsg(tf_map_to_base_link);

        double x = final_transform.translation.x;
        double y = final_transform.translation.y;
        double z = final_transform.translation.z;

        tf2::Quaternion q(final_transform.rotation.x, final_transform.rotation.y,
                          final_transform.rotation.z, final_transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      
        geometry_msgs::PointStamped tf_point_msg, ego_point_msg;
        tf_point_msg.header.stamp = ros::Time::now();
        tf_point_msg.header.frame_id = map_frame;  

        tf_point_msg.point.x = x;    // Set X coordinate
        tf_point_msg.point.y = y;    // Set Y coordinate
        tf_point_msg.point.z = 0.0;  // Set Z coordinate

        // Publish the point

        ROS_WARN("\t TF POSE [x: %f, y: %f, theta: %f]", tf_point_msg.point.x, tf_point_msg.point.y, yaw);
        point_pub.publish(tf_point_msg);

        ego_point_msg.header.stamp = ros::Time::now();
        ego_point_msg.header.frame_id = map_frame;  

        ego_point_msg.point.x = vehTranslationX;    // Set X coordinate
        ego_point_msg.point.y = vehTranslationY;    // Set Y coordinate
        ego_point_msg.point.z = 0.0;  // Set Z coordinate
        ROS_INFO("\t EGO POSE [x: %f, y: %f, theta: %f]", ego_point_msg.point.x, ego_point_msg.point.y, vehYaw);
          
        point_pub2.publish(ego_point_msg);



      }

    } catch (tf2::TransformException& ex) {
      ROS_WARN("Could not get transform: %s", ex.what());
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}