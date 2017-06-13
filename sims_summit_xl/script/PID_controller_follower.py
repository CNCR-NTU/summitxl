#!/usr/bin/env python

import rospy
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from sims_summit_xl.msg import pos as Position_Msg # definition of Position_Msg (msg/pos.msg)

class Follower:
	def __init__(self):
		self.controllerLossTimer = threading.Timer(1,self.controllerLoss)
		self.controllerLossTimer.start()
		self.switchMode = False
		self.max_speed = 0.5
		self.controllButtonIndex = -4 #red round button on the ps3 controller


		self.buttonCallbackBusy = False
		self.active = False

		self.cmd_vel_Publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

		self.joy_Subscriber = rospy.Subscriber('/joy',Joy, self.buttonCallback)

		self.position_Susbriber= rospy.Subscriber('/SIMS/position_data',Position_Msg, self.positionUpdateCallback)

		#PID Parameters
		set_point = 1.5
		PID_param = {'P': [0.7, 0.6],'I': [0.07,0.04],'D': [0.0, 0.0]}
		self.PID_controller = PID([0,set_point], PID_param['P'], PID_param['I'], PID_param['D'])

		rospy.on_shutdown(self.controllerLoss)

	def positionUpdateCallback(self,position):
		
		if(self.active):
			
			angleX = position.angleX
			distance = position.distance

			rospy.logwarn('Angle: {}, Distance: {}, '.format(angleX, distance))

			#CALLING PID CONTROLLER to update it and get new vectors of velocity
			[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])

			# clip these speeds to be less then the maximal speed specified above
			angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
			linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
			
			# create the Twist message to send to the cmd_vel topic
			velocity = Twist()
			velocity.linear = Vector3(linearSpeed,0,0.)
			velocity.angular= Vector3(0., 0.,angularSpeed)
			rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
			self.cmd_vel_Publisher.publish(velocity)

	def buttonCallback(self, joy_data):
		# this method gets called whenever we receive a message from the joy stick

		# there is a timer that always gets reset if we have a new joy stick message
		# if it runs out we know that we have lost connection and the controllerLoss function
		# will be called
		self.controllerLossTimer.cancel()
		self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
		self.controllerLossTimer.start()
		# if we are in switch mode, one button press will make the follower active / inactive 
		# but 'one' button press will be visible in roughly 10 joy messages (since they get published to fast) 
		# so we need to drop the remaining 9
		
		if self.buttonCallbackBusy:
			# we are busy with dealing with the last message
			return 
		else:
			# we are not busy. i.e. there is a real 'new' button press
			# we deal with it in a seperate thread to be able to drop the other joy messages arriving in the mean
			# time
			thread.start_new_thread(self.threadedButtonCallback,  (joy_data,))
	
	def threadedButtonCallback(self, joy_data):
		self.buttonCallbackBusy = True
		if(joy_data.buttons[13]==self.switchMode and self.active):
				# we are active
				# switchMode = false: we will always be inactive whenever the button is not pressed (buttons[index]==false)
				# switchMode = true: we will only become inactive if we press the button. (if we keep pressing it, 
				# we would alternate between active and not in 0.5 second intervalls)
				rospy.loginfo('stoping')
				self.stopMoving()
				self.active = False
				rospy.sleep(0.5)
		elif(joy_data.buttons[self.controllButtonIndex]==True and not(self.active)):
			# if we are not active and just pressed the button (or are constantly pressing it) we become active
			rospy.loginfo('activating')
			self.active = True #enable response
			rospy.sleep(0.5)

		self.buttonCallbackBusy = False

	def stopMoving(self):
		velocity = Twist()
		velocity.linear = Vector3(0.,0.,0.)
		velocity.angular= Vector3(0.,0.,0.)
		self.cmd_vel_Publisher.publish(velocity)

	def controllerLoss(self):
		# we lost connection so we will stop moving and become inactive
		self.stopMoving()
		self.active = False
		rospy.loginfo('lost connection')

	


class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, target, P, I, D):
		'''Create a discrete PID controller
		each of the parameters may be a vector if they have the same length
		
		Args:
		target (double) -- the target value(s)
		P, I, D (double)-- the PID parameter

		'''

		# check if parameter shapes are compatabile. 
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P,I,D))
		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		
		self.last_error=0
		self.integrator = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		
		Args:
			current_value (double): vector/number of same legth as the target given in the constructor

		Returns:
			controll signal (double): vector of same length as the target
		
		"""
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.clock()
			return np.zeros(np.size(current_value))

		
		error = self.setPoint - current_value
		P =  error
		
		currentTime = time.clock()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# derivative is difference in error / time since last update
		D = (error-self.last_error)/deltaT
		
		self.last_error = error
		self.timeOfLastCall = currentTime
		
		# return controll signal
		return self.Kp*P + self.Ki*I + self.Kd*D


if __name__ == '__main__':
	print('starting')
	rospy.init_node('PID_follower')
	follower = Follower()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')
