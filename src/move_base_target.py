#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class CentroidTransformer:
    def __init__(self):
        rospy.init_node('centroid_transformer', anonymous=True)
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher("/detection_target", PointCloud, queue_size=10)
        rospy.Subscriber("/detected_person_centroid", PointStamped, self.callback)
        self.current_point = None
        self.rate = rospy.Rate(10)  # 10Hz

    def callback(self, data):
        try:
            self.listener.waitForTransform("map", data.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            transformed_point = self.listener.transformPoint("map", data)
            print(transformed_point)
            rospy.loginfo(f"Centroid in map frame: {transformed_point.point}")

            # Update current point to the new transformed point
            self.current_point = Point32(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)

    def publish_point(self):
        if self.current_point is not None:
            point_cloud = PointCloud()
            point_cloud.header.stamp = rospy.Time.now()
            point_cloud.header.frame_id = "map"
            point_cloud.points = [self.current_point]
            self.publisher.publish(point_cloud)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_point()
            self.rate.sleep()

if __name__ == '__main__':
    transformer = CentroidTransformer()
    transformer.run()
