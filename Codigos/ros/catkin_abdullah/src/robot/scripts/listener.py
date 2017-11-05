#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from arduino_msgs.msg import StampedFloat32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', StampedFloat32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
