#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Polygon, Point32

def polygon_publisher():
    # Initialize the ROS node
    rospy.init_node('polygon_publisher', anonymous=True)
    
    # Create a publisher object
    pub = rospy.Publisher('/predicted_footprint', PolygonStamped, queue_size=10)
    
    # Set the rate at which to publish the messages (10 Hz)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Create a Polygon message
        polygonStamped = PolygonStamped()

        polygonStamped.header.stamp = rospy.Time.now()

        # Create points to define the polygon (e.g., a triangle)
        point1 = Point32()
        point1.x = 0.0
        point1.y = 0.0
        point1.z = 0.0
        
        point2 = Point32()
        point2.x = 1.0
        point2.y = 0.0
        point2.z = 0.0
        
        point3 = Point32()
        point3.x = 0.5
        point3.y = 1.0
        point3.z = 0.0

        # Add points to the polygon
        polygonStamped.polygon.points.append(point1)
        polygonStamped.polygon.points.append(point2)
        polygonStamped.polygon.points.append(point3)

        # Log the message
        #rospy.loginfo(f"Publishing Polygon with points: {polygon.points}")
        
        # Publish the message
        pub.publish(polygonStamped)
        
        # Sleep to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        polygon_publisher()
    except rospy.ROSInterruptException:
        pass
