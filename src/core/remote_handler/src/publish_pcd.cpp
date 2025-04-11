#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string pcd_file_path, pcd_pub_topic, pcd_frame_id; 
    double pub_frequency;
    private_nh.getParam("pcd_file_path", pcd_file_path);
    private_nh.getParam("pcd_pub_topic", pcd_pub_topic);
    private_nh.getParam("pcd_frame_id", pcd_frame_id);
    private_nh.getParam("pub_frequency", pub_frequency);
    ROS_INFO_STREAM("pcd_file_path: "<<pcd_file_path);
    ROS_INFO_STREAM("pcd_pub_topic: "<<pcd_pub_topic);
    ROS_INFO_STREAM("pcd_frame_id: "<<pcd_frame_id);
    ROS_INFO_STREAM("pub_frequency: "<<pub_frequency);

    // Publisher for the point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(pcd_pub_topic, 1);

    
    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file \n");
        return -1;
    }

    // Convert the PCL point cloud to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);

    // Set the frame ID
    output.header.frame_id = pcd_frame_id;

    // Publish the point cloud at a regular interval
    ros::Rate loop_rate(pub_frequency); 
    while (ros::ok())
    {
        output.header.stamp = ros::Time::now(); // Update timestamp
        pub.publish(output); // Publish the message

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}