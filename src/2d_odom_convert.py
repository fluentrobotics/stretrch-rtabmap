#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def odom_callback(data):
    # x, y positions and quaternion orientation
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    )

    # quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = euler[2]  # yaw 


    odom_2d = Odometry()
    odom_2d.header = data.header  
    odom_2d.child_frame_id = data.child_frame_id
    odom_2d.pose.pose.position.x = x
    odom_2d.pose.pose.position.y = y
    odom_2d.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))


    odom_2d.twist = data.twist

    pub.publish(odom_2d)

    rospy.loginfo("Timestamp: {} - x: {}, y: {}, theta: {}".format(
        data.header.stamp, x, y, theta))


def listener():
    rospy.init_node('odom_2d_converter', anonymous=True)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/rtabmap/odom2d', Odometry, queue_size=10)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
