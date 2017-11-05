#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from arduino_msgs.msg import StampedFloat32

def talker():
    pub1 = rospy.Publisher('testint', StampedFloat32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    test = StampedFloat32()
    while not rospy.is_shutdown():
	    test.id = 10
	    test.data = 20
        pub1.publish(test)
	    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass