#!/usr/bin/env python
# test mail: chutter@uos.de

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from simple_follower.msg import position as PositionMsg
		
class laserDetector:
	def __init__(self):
		self.lastScan=None
		self.winSize = 2
		self.deltaDist = 0.2		
		self.scanSubscriber = rospy.Subscriber('/hokuyo_base/scan', LaserScan, self.dataProcess)

	def dataProcess(self, data):
		#min and max range detection [m]
		self.minDetectableRange = data.range_min
		self.maxDetectableRange = data.range_max
		#putting the ranges array in a numpy array to use it easily
		ranges = np.array(data.ranges)
		# sort by distance/ranges from closer to further away
		sortedRanges = np.argsort(ranges)
		
		minCurrentRangeID = -1
		minCurrentRange   = float('inf')
          
            		

		if(not(self.lastScan is None)):
			#if we already have a scan processed
			for i in sortedRanges:
				#processing all ranges
				tmpRange = ranges[i]				
				#checking if there's a noise
				#defining the window of value we want 
				#checking if the ranges detected were in the lastScan and within that window
				
				#set value in a range with np.clip
				windowIndex = np.clip([i-self.winSize, i+self.winSize+1],0,len(self.lastScan))
				window = self.lastScan[windowIndex[0]:windowIndex[1]]

				with np.errstate(invalid='ignore'):

					if(np.any(abs(window-tmpRange)<=self.deltaDist)):
					# this will also be false for all tmpRange = inf

						# we found a plausible distance
						minCurrentRangeID = i
						minCurrentRange = ranges[minCurrentRangeID]
						break # at least one point was equally close
      
						# so we found a valid minimum and can stop the loop
			
		self.lastScan=ranges	
		
		#catches nothing to scan (if the min range is inf then condition is true)
		if(minCurrentRange > self.maxDetectableRange):
			
			rospy.logwarn('laser no object found')
			
		else:
			# calculate angle of the objects location. 0 is straight ahead
			minCurrentRangeAngle = data.angle_min + minCurrentRangeID * data.angle_increment
			# here we only have an x angle, so the y is set arbitrarily			
			rospy.logwarn(PositionMsg(minCurrentRangeAngle, minCurrentRange))
			



if __name__ == '__main__':
	rospy.init_node('laser_Detector')
	tracker = laserDetector()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


