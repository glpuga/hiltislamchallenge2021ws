#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthScaler:

  def __init__(self):
    self._bridge = CvBridge()

    self._encoding = "16UC1"

    # The 1.14 value was obtained by comparing laser and depth cloud measurements
    self._depth_scale = rospy.get_param("depth_scale_factor", 1.14)

    self._output_pub = rospy.Publisher("output_depth_image", Image)
    self._input_sub = rospy.Subscriber("input_depth_image", Image, self._callback)

  def run(self):
      rospy.spin()

  def _callback(self, msg):
    try:
      cv_image = self._bridge.imgmsg_to_cv2(msg, self._encoding)
      
      cv_image = (cv_image * self._depth_scale).astype(np.uint16)

      
      new_img = self._bridge.cv2_to_imgmsg(cv_image, self._encoding)
      new_img.header = msg.header
      self._output_pub.publish(new_img)
    except Exception as e:
      rospy.logerr(e)


def main():
  rospy.init_node('depth_scaler', anonymous=True)
  try:
    node = DepthScaler()
    node.run()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()