#!/usr/bin/python3
import rospy

from cartographer_ros_msgs.srv import TrajectoryQuery


class Recorder(object):
    def __init__(self):

        self._autosave_interval = rospy.get_param(
            "~autosave_interval", 3)
        self._output_file = rospy.get_param(
            "~trajectory_file")

        self._trajectory_service_name = "/trajectory_query"
        rospy.wait_for_service(self._trajectory_service_name)
        self._get_trajectory = rospy.ServiceProxy(
            self._trajectory_service_name, TrajectoryQuery)

        rospy.Timer(rospy.Duration(self._autosave_interval),
                    self._timer_callback)

    def _timer_callback(self, event):
        rospy.loginfo("Recovering trajectory data from Cartographer...")
        response_msg = self._get_trajectory(trajectory_id=0)

        # convert trajectory to output format ready to save to file
        output_file_lines = [
            "{} {} {} {} {} {} {} {}\n".format(
                x.header.stamp.to_sec(),  # timestamp
                x.pose.position.x, x.pose.position.y, x.pose.position.z,  # position
                x.pose.orientation.w, x.pose.orientation.y, x.pose.orientation.z, x.pose.orientation.w  # orientation
            )
            for x in response_msg.trajectory]

        # Open the output file and save it
        rospy.loginfo("Saving trajectory to file...")
        with open(self._output_file, "w") as file_desc:
            file_desc.writelines(output_file_lines)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('data_recorder')
    Recorder().run()


main()
