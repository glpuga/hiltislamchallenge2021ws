#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Imu


class MergeImuData(object):
    def __init__(self):
        self._accel_topic = rospy.get_param("~accel_topic", 'accel')
        self._gyro_topic = rospy.get_param("~gyro_topic", 'gyro')
        self._combined_topic = rospy.get_param("~combined_topic", 'combined')

        self._latest_accel_msg = None

        # subscribers
        self._accel_sub = rospy.Subscriber(self._accel_topic,
                                           Imu,
                                           self._accel_callback, queue_size=20)
        self._gyro_sub = rospy.Subscriber(self._gyro_topic,
                                          Imu,
                                          self._gyro_callback, queue_size=20)

        # publisher
        self._combined_pub = rospy.Publisher(self._combined_topic,
                                             Imu, queue_size=20)

    def _accel_callback(self, msg):
        self._latest_accel_msg = msg

    def _gyro_callback(self, msg):
        if self._latest_accel_msg:
            msg.linear_acceleration = self._latest_accel_msg.linear_acceleration
            msg.linear_acceleration_covariance = self._latest_accel_msg.linear_acceleration_covariance
        self._combined_pub.publish(msg)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('node_name')
    MergeImuData().run()


main()
