#!/usr/bin/env python
import roslib; #roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
#from motherbrain.srv import *

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

class Lift(object):
	def __init__(self, nbRobot):
		
		
		rospy.loginfo('Waiting for the service in lift')
		self.serviceLift = list()
		self.nbRobot=nbRobot
		self.flag=False
		try:
			i=0
			if nbRobot>1:
				while i<nbRobot:
					name='robot'+str(i)+'/lifting_service'
					rospy.wait_for_service(name)
					i=i+1
					self.serviceLift.append(rospy.ServiceProxy(name, lifting) )
			else:
				name='/lifting_service'
				rospy.wait_for_service(name)
				i=i+1
				self.serviceLift.append(rospy.ServiceProxy(name, lifting) )
		except rospy.ServiceException, e:
				print "Service call failed: %s"%e
		
	def execute(self):
		rospy.loginfo('Executing state Lifting for the target')
		
		flag=1
		
		for elm in self.serviceLift:
			print 'flag '+str(flag)
			if self.flag == True:
				#Do processing to unlift
				rospy.loginfo('Unlift')
				try:
					rep=elm('down')
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
				if(rep.answer==True) and flag==self.nbRobot:
					#userdata.end_object_flag=False
					print('The object is now UN-lifted')
					self.flag=False
					return 'valid_unlift'
				elif (rep.answer==True) and flag<self.nbRobot:
					flag=flag+1
				elif (rep.answer==True) and flag>self.nbRobot:
					return 'invalid'
				else:
					return 'invalid'
				
			else:
				#Do processing to lift
				rospy.loginfo('Lift')
				try:
					rep=elm('up')
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
				if(rep.answer==True) and flag==self.nbRobot:
					#userdata.end_object_flag=True
					print('The object is now lifted')
					self.flag=True
					return 'valid'
				elif (rep.answer==True) and flag<self.nbRobot:
					flag=flag+1
				elif (rep.answer==True) and flag>self.nbRobot:
					return 'invalid'
				else:
					return 'invalid'
			
		return 'invalid'
#		rospy.loginfo('done. We have : ' + str(userdata.flag) )
		





moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1),
		'v':(1000,4),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	th = 0
	status = 0
	
	#lift = Lift(1)

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][1]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				th = 0
				if (key == '\x03'):
					break

			if x !=1000:
				twist = Twist()
				twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				pub.publish(twist)
			else:
				lift.execute()
				x=0
				th=0

	except Exception, e:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


