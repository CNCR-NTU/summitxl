#!/usr/bin/env python

import rospy
import message_filters # filter the data send by camera (used for synchronizing frames)
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError # bridge for converting ROS Messages to OpenCV encoding and reverse
from sensor_msgs.msg import Image # encoding type of ROS Images
from sims_summit_xl.msg import pos as Position_Msg
from matplotlib import pyplot as plt

np.seterr(all='raise')  

class cameraDetector:
	'''Initializing bridge subscriber and Publisher and getting data 
	from camera and synchronizing rgb and depth frames'''
	def __init__(self):
		self.bridge = CvBridge()
		RGB_img = message_filters.Subscriber('/orbbec_astra/rgb/image_raw', Image)
		depth_img = message_filters.Subscriber('/orbbec_astra/depth/image_raw', Image)
		self.TimeSync = message_filters.TimeSynchronizer([RGB_img,depth_img],10)
		self.TimeSync.registerCallback(self.tracking) # callback function tracking (sending image data towards it)
		self.image_publisher = rospy.Publisher('SIMS/image_topic',Image, queue_size=3)
		self.pos_publisher = rospy.Publisher('SIMS/position_data',Position_Msg,queue_size=3)

		''' Initializing all variables needed'''
		self.lastPos = None
		self.image_height = 0.0
		self.image_width = 0.0
		# getting tangens from field of view values (stored in .launch file)
		self.tan_horizontal = np.tan(rospy.get_param('~angle_of_view/horizontal'))
		self.tan_vertical = np.tan(rospy.get_param('~angle_of_view/vertical'))

	'''Main function of tracking'''
	def tracking(self,rgb_data,depth_data):

		# getting the images dimensions
		self.image_height = rgb_data.height
		self.image_width = rgb_data.width

		# transforming ROS Messages data to OpenCV data 
		try:
			cv_rgb = self.bridge.imgmsg_to_cv2(rgb_data,desired_encoding='rgb8')
			cv_depth = self.bridge.imgmsg_to_cv2(depth_data,desired_encoding = 'passthrough')
		except CvBridgeError as e :
			print(e)
		
		# define range of blue color in HSV
		lower = np.array([0, 100,100])
		upper = np.array([40, 255, 255])
		
		# blur image and convert to HSV color encoding
		blurred = cv2.GaussianBlur(cv_rgb,(9,9),0)
		hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
		
		# HSV image to get only blue colors
		mask1 = cv2.inRange(hsv,lower,upper)
		
		# reducing the noise by eroding an then dilating, the iterations are arbitrary
		mask2 = cv2.erode(mask1, None, iterations=3)
		mask3 = cv2.dilate(mask2,None, iterations=3)
		
		# find contours of the object
		cnts = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

		# checking if any object was found
		if len(cnts)==0:
			rospy.logwarn('no object found')

		# loop for each contour of contours
		for cnt in sorted(cnts, key=cv2.contourArea, reverse=True):

			#getting the pos
			pos = self.getPos_contour(cnt,cv_depth)
			if self.checkPos(pos):
				self.lastPos = pos 
				# getting angles from pos
				angleX = self.getAngleX(pos)
				angleY = self.getAngleY(pos)
				#creating the Position_Msg and publishing it
				pos_Msg = Position_Msg(angleX,angleY,pos[1])
				rospy.logwarn(pos_Msg)
				self.pos_publisher.publish(pos_Msg)
			else:
				rospy.logwarn('no plausible position found')



			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(cv_rgb,[box],0,(0,0,255),2)

			
			
		#publishing on a topic which can be seen in rviz for example
		try:
			self.image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_rgb,rgb_data.encoding))
		except CvBridgeError as e:
			print(e)

	''' Return Center position and the average distance to the object detected
	
		Parameters :
			cnt : contour of an object detected (type : OpenCV contour)
			cv_depth : depth array of the image (type : OpenCV image format / numpy array)

		Returns : pos (type: array of array)
		pos[0] = (CenterX,CenterY) : coordinates of the center of object detected (type:array of 2 float)
		pos[1] = averageDistance (type : float)

	'''
	def getPos_contour(self,cnt,cv_depth):
		# getting the center(float coordinates), size and rotation of a rectangle which contains the contour
		centerFloat, size, rotation = cv2.minAreaRect(cnt)

		# rounds the coordinates of the center and cast it into an int
		centerInt = np.round(centerFloat).astype(int)

		# casting into ints and reducing the size of the contour to be more precise (arbitrary)
		xSizeMin = size[0]
		xSizeMin = int(xSizeMin/2)
		ySizeMin = size[1]
		ySizeMin = int(ySizeMin/2)

		# getting the depth points corresponding to the rectangle obtained from the contour
		depthObject = cv_depth[(centerInt[0]-xSizeMin):(centerInt[0]+xSizeMin),(centerInt[1]-ySizeMin):(centerInt[1]+ySizeMin)]

		# getting rid of all Nan values which are not useful
		depthArray = depthObject[~np.isnan(depthObject)]
		
		# np.mean gives the average of the array elements
		averageDistance = np.mean(depthArray)

		if len(depthArray) == 0:
			rospy.logwarn('empty depth array. all depth values are nan')
				
		return (centerFloat,averageDistance)
		
		

	''' Return True if pos is plausible return the opposit if not '''
	def checkPos(self,pos):
		ret = True


		# for the first scan we don't have a LastScan to compare => return false to set the LastScan
		if self.lastPos is None:
			self.lastPos = pos  #setting lastPos
			ret = False

		# unpack pos
		((centerX, centerY),distance) = pos
		((LcenterX, LCenterY),Ldistance) = self.lastPos

		if np.isnan(distance):
			ret = False
		
		# testing the distance value
		if abs(distance-Ldistance)>0.4:
			ret = False
		
		# testing the coordinates 
		if abs(centerX-LcenterX)>(self.image_width/6):
			ret = False
		if abs(centerY-LCenterY)>(self.image_height/6):
			ret = False

		return ret

	''' Return the X angle of displacement to the objet tracked '''
	def getAngleX(self,pos):
		centerX = pos[0][0]
		displacement = 2*centerX/self.image_width-1
		angleX = -1*np.arctan(displacement*self.tan_horizontal)
		return angleX

	''' Return the Y angle of displacement'''
	def getAngleY(self,pos):
		centerY = pos[0][1]
		displacement = 2*centerY/self.image_height-1
		angleY = -1*np.arctan(displacement*self.tan_vertical)
		return angleY

		
if __name__ == '__main__':
	rospy.init_node('camera_detector')
	detector = cameraDetector()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')

