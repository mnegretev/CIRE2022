#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

global new_sensor

new_sensor=LaserScan()

def callback_Laser(msg):
	global new_sensor
	new_sensor=msg
	#new_sensor.header.stamp=rospy.Time.now()
	new_sensor.ranges=list(new_sensor.ranges)
	#print(type(new_sensor.ranges))
	for i in range(481-20,481+25):
		new_sensor.ranges[i]=math.inf

def main():
	
	global original_sensor, new_sensor
	
	print("Initializing sensor_scan_fix")
	rospy.init_node('sensor_scan_fix', anonymous=True)
	rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_Laser)
	pub_sensor=rospy.Publisher('/hsrb/base_scan/fix', LaserScan, queue_size=1)
	loop=rospy.Rate(38)

	while not rospy.is_shutdown():
		pub_sensor.publish(new_sensor)
		loop.sleep()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass