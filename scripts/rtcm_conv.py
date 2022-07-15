#!/usr/bin/env python3

"""
Converts mavros_msgs::RTCM to rtcm_msgs::Message
Usually used for ublox ros driver with input type rtcm_msgs::Message
"""

import sys
import rospy
from rtcm_msgs.msg import Message
from mavros_msgs.msg import RTCM

class RTCMConversion:
    def __init__(self):
        self.pub = rospy.Publisher('/rtcm', Message, queue_size=10)
        self.sub = None
        rospy.init_node('rtcm_conv', anonymous=False)
        self.rtcm_in_topic = rospy.get_param('~rtcm_topic', 'rtcm')
        rospy.loginfo("rtcm_in_topic: " + self.rtcm_in_topic)

    def callback(self, data):
        rtcm = Message()
        rtcm.header = data.header
        rtcm.message = data.data
        self.pub.publish(rtcm)

    def run(self):
        # Setup a shutdown hook
        rospy.on_shutdown(self.stop)

        self.sub = rospy.Subscriber(self.rtcm_in_topic, RTCM, self.callback)

        # Spin until we are shutdown
        rospy.spin()
        return 0

    def stop(self):
        if self.sub: self.sub.unregister()

if __name__ == "__main__":
    conv = RTCMConversion()
    sys.exit(conv.run())

