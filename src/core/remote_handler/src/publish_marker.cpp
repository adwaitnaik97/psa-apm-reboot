#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_array_publisher");
  ros::NodeHandle nh;

  // Publisher for MarkerArray
  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("sample_visualization_marker_array", 1);

  ros::Rate loop_rate(1);  // Publish at 1 Hz

  while (ros::ok())
  {
    visualization_msgs::MarkerArray marker_array;

    // Create individual markers and add them to the array
    for (int i = 0; i < 3; ++i)  // Example: 3 markers
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "base_link";  // Set the frame in which markers are displayed
      marker.header.stamp = ros::Time::now();
      marker.ns = "basic_shapes";
      marker.id = i;  // Each marker must have a unique ID in the same namespace

      // Set the marker type: a sphere in this case
      marker.type = visualization_msgs::Marker::SPHERE;

      // Set the position of the marker
      marker.pose.position.x = i;
      marker.pose.position.y = i * 2.0;
      marker.pose.position.z = 0.0;

      // Set orientation (quaternion)
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale (size) of the marker
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      // Set the color of the marker (RGBA)
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;  // Fully opaque

      // Set the lifetime of the marker
      marker.lifetime = ros::Duration();  // Forever

      // Add the marker to the MarkerArray
      marker_array.markers.push_back(marker);
    }

    // Publish the MarkerArray
    marker_array_pub.publish(marker_array);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}