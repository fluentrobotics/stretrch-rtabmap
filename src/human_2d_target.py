#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped

def callback(data):
    rospy.loginfo(f"Received centroid at x: {data.point.x}, y: {data.point.y}, z: {data.point.z}")

def listener():
    rospy.init_node('centroid_listener', anonymous=True)
    rospy.Subscriber("/detected_person_centroid", PointStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
