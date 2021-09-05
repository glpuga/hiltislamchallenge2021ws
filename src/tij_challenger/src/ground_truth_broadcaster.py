#!/usr/bin/env python
import rospy

import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class GroundTruthTransformPublisher(object):
    def __init__(self):
        # transform broadcaster
        self._tf_broadcaster = tf.TransformBroadcaster()

        # Odometry publisher, for the benefit being able to show
        # the trace in RVIZ
        self._odometry_pub = rospy.Publisher(
            "/vicon_client/METRICS/ideal_odom", Odometry, queue_size=20)

        # subscriber
        self._pose_sub = rospy.Subscriber('/vicon_client/METRICS/pose',
                                          PoseStamped,
                                          self._pose_callback, queue_size=20)

    def _pose_callback(self, msg):
        self._tf_broadcaster.sendTransform(
            (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w),
            rospy.Time.now(),
            "ground_truth",
            "vicon")
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.pose.pose = msg.pose
        self._odometry_pub.publish(odom_msg)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('node_name')
    GroundTruthTransformPublisher().run()


main()
