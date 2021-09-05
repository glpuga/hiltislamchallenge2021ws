#!/usr/bin/env python
import rospy

import tf2_ros as tf2
from geometry_msgs.msg import PoseStamped


class LogFile(object):
    def __init__(self, filepath):
        self._fd = open(filepath, "w+")

    def write(self, line):
        self._fd.write(line + "\n")

    def __del__(self):
        self._fd.close()


class Recorder(object):
    def __init__(self):

        self._odom_frame = "odom"
        self._base_link_frame = "base_link"

        self._traj_sampling_interval = rospy.get_param(
            "traj_sampling_interval", 0.1)

        self._gt_topic = rospy.get_param(
            "~ground_truth_topic", '/vicon_client/METRICS/pose')

        self._log_ground_truth = rospy.get_param("~log_ground_truth", True)
        self._log_stamped_traj = rospy.get_param("~log_stamped_traj", True)

        self._stamped_ground_truth_file = rospy.get_param(
            "~stamped_ground_truth_file", "")
        self._stamped_traj_file = rospy.get_param("~stamped_traj_file", "")

        self._tf_buffer = tf2.Buffer()
        self._tf_listener = tf2.TransformListener(self._tf_buffer)

        if (self._log_ground_truth and not self._stamped_ground_truth_file):
            rospy.logerr(
                "To log the stamped ground truth you need to specify the filepath")
            raise ValueError()

        if (self._log_stamped_traj and not self._stamped_traj_file):
            rospy.logerr(
                "To log the stamped trajectory you need to specify the filepath")
            raise ValueError()

        if self._log_stamped_traj:
            self._traj_logger = LogFile(self._stamped_traj_file)
            rospy.Timer(rospy.Duration(
                self._traj_sampling_interval), self._timer_callback)

        if self._log_ground_truth:
            self._gt_logger = LogFile(self._stamped_ground_truth_file)
            self._pose_sub = rospy.Subscriber(self._gt_topic,
                                              PoseStamped,
                                              self._pose_callback, queue_size=20)

    def _timer_callback(self, event):
        try:
            trans = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_link_frame, rospy.Time())
            line = "{} {} {} {} {} {} {} {}".format(
                trans.header.stamp.to_sec(),
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w)
            self._traj_logger.write(line)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            rospy.logerr("Unable to get the tranform from {} to {}".format(
                self._odom_frame, self._base_link_frame))

    def _pose_callback(self, msg):
        line = "{} {} {} {} {} {} {} {}".format(
            msg.header.stamp.to_sec(),
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w)
        self._gt_logger.write(line)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('node_name')
    Recorder().run()


main()
