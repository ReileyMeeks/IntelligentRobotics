#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('my_name', String, queue_size=10)
	rospy.init_node('Reiley_Meeks', anonymous=True)
	rate = rospy.Rate(5) #5hz
	
	while not rospy.is_shutdown():
		hellostr = "Reiley Meeks %s" % rospy.get_time()
		rospy.loginfo(hellostr)
		pub.publish(hellostr)
		rate.sleep()
		
if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
