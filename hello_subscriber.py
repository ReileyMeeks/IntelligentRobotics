#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Hello World %s", data.data)
	
def listener():
	rospy.init_node('Hello_Subscriber', anonymous=True)
	rospy.Subscriber("my_name", String, callback)
	rospy.spin()
	
if __name__=='__main__':
	listener()
