#!/usr/bin/env python

# Python libs
import sys
import time
import math
import random
import actionlib
import subprocess
import signal

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import smach
import smach_ros

# Ros Messages
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from exp_assignment3.msg import Ball


black_ball = Ball()
red_ball = Ball()
yellow_ball = Ball()
green_ball = Ball()
blue_ball = Ball()
magenta_ball = Ball()

ball_room = Ball()

VERBOSE = False 

# define state Normal
class Normal(smach.State):
    """
    In NORMAL state we have launched the explore_Lite package. While the timerdoesn't exceed a certain time, the robot keeps exploring the environment. While this timer doesn't end and the user gives the play command, ther robot enters into the PLAY state, else the robot goes into the SLEEP behavior. While the explore_lite package is running, the NORMAL also executes a function, detectBall(), to detect any ball in the environment.
    """

    def __init__(self):
        
        smach.State.__init__(self, 
                             outcomes=['after finishing normal behavior',
 				       'speech command:play'])

    def execute(self, userdata):
	
	global child, pub_cancel, playTimer
	cancel_msg = GoalID()
	rospy.loginfo('Executing state NORMAL')
	child = subprocess.Popen(["roslaunch","exp_assignment3","explore.launch"])
	rospy.set_param("expLiteFlag", 1) 
	timer = time.clock()
	while(time.clock() - timer < 240):
		if (rospy.get_param("playflag") == 1):
			child.send_signal(signal.SIGINT)
			pub_cancel.publish(cancel_msg)
			playTimer = time.clock()
			return 'speech command:play'
		detectBall()
	child.send_signal(signal.SIGINT)
	return 'after finishing normal behavior'


def robotPos(currentPos):
    
    global robot_x,robot_y
    robot_x = currentPos.pose.pose.position.x
    robot_y = currentPos.pose.pose.position.y

def imageCallback(image):
	
	global np_arr
        if VERBOSE:
            print ('received image of type: "%s"' % image.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(image.data, np.uint8)

def clbk_laser(msg):
    global regions_

    regions_ = {
	'front_right': min(min(msg.ranges[180:360]), 10),
	'front_left': min(min(msg.ranges[361:540]), 10),
    }


def detectBall():
	
        """
	It uses Open CV to process the image received, and detects the contours of the different colored balls in the image using different masks for each color. 
	If any contour is found, it passes the radius and center of the detected countour, and the detected ball object to the function track(). track() is implemented for going near the ball after shutting down the explore_lite package.  
	"""
	
	global np_arr, child
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

	blackLower = (0, 0, 0) 
	blackUpper = (5,50,50)
	redLower = (0, 50, 50)
	redUpper = (5, 255, 255)
	yellowLower = (25, 50, 50) 
	yellowUpper = (35, 255, 255)
	greenLower = (50, 50, 50) 
	greenUpper = (70, 255, 255)
	blueLower = (100, 50, 50) 
	blueUpper = (130, 255, 255)
	magentaLower = (125, 50, 50) 
	magentaUpper = (150, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	mask_blk = cv2.inRange(hsv, blackLower, blackUpper)
        mask_blk = cv2.erode(mask_blk, None, iterations=2)
        mask_blk = cv2.dilate(mask_blk, None, iterations=2)

	mask_r = cv2.inRange(hsv, redLower, redUpper)
        mask_r = cv2.erode(mask_r, None, iterations=2)
        mask_r = cv2.dilate(mask_r, None, iterations=2)

	mask_y = cv2.inRange(hsv, yellowLower, yellowUpper)
        mask_y = cv2.erode(mask_y, None, iterations=2)
        mask_y = cv2.dilate(mask_y, None, iterations=2)

	mask_g = cv2.inRange(hsv, greenLower, greenUpper)
        mask_g = cv2.erode(mask_g, None, iterations=2)
        mask_g = cv2.dilate(mask_g, None, iterations=2)

        mask_blu = cv2.inRange(hsv, blueLower, blueUpper)
        mask_blu = cv2.erode(mask_blu, None, iterations=2)
        mask_blu = cv2.dilate(mask_blu, None, iterations=2)

	mask_m = cv2.inRange(hsv, magentaLower, magentaUpper)
        mask_m = cv2.erode(mask_m, None, iterations=2)
        mask_m = cv2.dilate(mask_m, None, iterations=2)
        #cv2.imshow('mask', mask)

        cnts_blk = cv2.findContours(mask_blk.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_r = cv2.findContours(mask_r.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_y = cv2.findContours(mask_y.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_g = cv2.findContours(mask_g.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_blu = cv2.findContours(mask_blu.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_m = cv2.findContours(mask_m.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts_blk = imutils.grab_contours(cnts_blk)
	cnts_r = imutils.grab_contours(cnts_r)
	cnts_y = imutils.grab_contours(cnts_y)
	cnts_g = imutils.grab_contours(cnts_g)
	cnts_blu = imutils.grab_contours(cnts_blu)
	cnts_m = imutils.grab_contours(cnts_m)

        center = None
	c = 0
	radius = 0

	global black_ball, red_ball, yellow_ball, green_ball, blue_ball, magenta_ball

        # only proceed if at least one contour was found
        if len(cnts_blk) > 0 and black_ball.detected_flag != True:
	    child.send_signal(signal.SIGINT)
            c = max(cnts_blk, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    black_ball = track(radius, center, black_ball)
	    print ("Black ball detected. Location: bedroom\n")

	elif len(cnts_r) > 0 and red_ball.detected_flag != True:
	    child.send_signal(signal.SIGINT)
            c = max(cnts_r, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    red_ball = track(radius, center, red_ball)
	    print ("Red ball detected. Location: closet\n")
	    
	elif len(cnts_y) > 0 and yellow_ball.detected_flag != True:
	    child.send_signal(signal.SIGINT)
            c = max(cnts_y, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    yellow_ball = track(radius, center, yellow_ball)
	    print ("Yellow ball detected. Location: kitchen\n")

	elif len(cnts_g) > 0 and green_ball.detected_flag != True:
	    child.send_signal(signal.SIGINT)
            c = max(cnts_g, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    green_ball = track(radius, center, green_ball)
	    print ("Green ball detected. Location: living room\n")

	elif len(cnts_blu) > 0 and blue_ball.detected_flag != True:
	    child.send_signal(signal.SIGINT)
            c = max(cnts_blu, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    blue_ball = track(radius, center, blue_ball)
	    print ("Blue ball detected. Location: entrance\n")

	elif len(cnts_m) > 0 and magenta_ball.detected_flag != True:
	    child.send_signal(signal.SIGINT)
            c = max(cnts_m, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    magenta_ball = track(radius, center, magenta_ball)
	    print ("Magenta ball detected. Location: bathroom\n")

	elif (rospy.get_param("expLiteFlag") == 0):
	    child = subprocess.Popen(["roslaunch","exp_assignment3","explore.launch"])
	    rospy.set_param("expLiteFlag", 1)

	cv2.imshow('window', image_np)
	cv2.waitKey(2)
	c = 0

def track(radius, center, detected_ball):
	"""
	This function is used to associate the coordinate to every ball and hence every room, in a way that it records its own location as the ball location, once the robot has reached quite near to the ball. When the explore_lite package gets shut down, the robot approaches the ball while avoiding obstacles and when a certain threshold of the radius of the ball has been reached the robot stops and the explore_lite package is activated again.  
	"""

	global child, regions_
	cancel_msg = GoalID()
	rospy.set_param("expLiteFlag", 0)
	pub_cancel.publish(cancel_msg)
	vel = Twist()
	
	if (radius > 90):
		print 'Reached'
		vel.angular.z = 0.0
		vel.linear.x = 0.0
		pub_cmd_vel.publish(vel)
		detected_ball.detected_flag = True
		detected_ball.location.x = robot_x
		detected_ball.location.y = robot_y
		print('Ball x: ', detected_ball.location.x,' Ball y: ', detected_ball.location.y)
		child = subprocess.Popen(["roslaunch","exp_assignment3","explore.launch"])
	        rospy.set_param("expLiteFlag", 1)

	elif (regions_['front_right'] > 0.5 and regions_['front_left'] > 0.5):
		vel.angular.z = -0.005*(center[0]-400)
		vel.linear.x = -0.007*(radius-100)
		pub_cmd_vel.publish(vel)

	else:
		if (regions_['front_right'] < 0.5):
			vel.angular.z = 0.2
			vel.linear.x = 0.0			
			pub_cmd_vel.publish(vel)

		if (regions_['front_left'] < 0.5):
			vel.angular.z = -0.2
			vel.linear.x = 0.0
			pub_cmd_vel.publish(vel)

	return detected_ball

# define state Sleep
class Sleep(smach.State):
    """
    If no play command is given while the robot is in the NORMAL state, the robot switches to the SLEEP state. This state is implemented by publishing (0,0) coordinates to move_base server where the robot stays for 5-10 seconds and then switches to the NORMAL state. 
    """

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['after finishing sleep time'])
        
    def execute(self, userdata):
        
	cv2.destroyAllWindows()
        rospy.loginfo('Executing state SLEEP')
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 0.0
	goal.target_pose.pose.position.y = 0.0
	goal.target_pose.pose.orientation.w = 1.0
	result = movebase_client(goal)
        if result:
           rospy.loginfo("Reached Sleep location!")
	time.sleep(random.randint(5,10))
	return 'after finishing sleep time'

def movebase_client(goal):

    """
    This function serves as a move_base client. It publishes goal to the move_base server and wait until the target has been reached.
    """

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

# define state Play
class Play(smach.State):
    """
    For PLAY state we implemented the Timer as a global variable because we wanted this state to stop after a certain time. This is done in this way becuase PLAY state will repeat more than once and we did not want Timer to be initialized again. 
    After giving play command in the GUI, the PLAY state of the robot is activated. First the robot moves near the human and waits for the GoTo location from the GUI. If the ball corresponding to the location has already been detected, the robot goes towards that ball or else the robot switches to the FIND state.  
    """

    def __init__(self): 
        
        smach.State.__init__(self, 
                             outcomes=['after finishing play time',
					'ball not already detected'])
        
    def execute(self, userdata):

        global ball_room, playTimer
	room = ""
	cv2.destroyAllWindows()
        rospy.loginfo('Executing state PLAY')
	ball_room = Ball()
	goal = MoveBaseGoal()

	while(time.clock() - playTimer < 300):
		rospy.set_param("playflag", 1)
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = -5.0
		goal.target_pose.pose.position.y = 8.0
		goal.target_pose.pose.orientation.w = 1.0
		result = movebase_client(goal)
		if result:
			rospy.set_param("room", "")
			rospy.set_param("nearHuman", 1)
		
		while(room == ""):
			room = rospy.get_param("room")
		rospy.set_param("room", "")
		if (room == "entrance"):
			if(blue_ball.detected_flag == True):
				ball_room = blue_ball
				rospy.set_param("room", "")

			else:
				rospy.set_param("ballToBeFound", "blue")
				return 'ball not already detected'
		elif (room == "closet"):
			if(red_ball.detected_flag == True):
				ball_room = red_ball
				rospy.set_param("room", "")
			else:
				rospy.set_param("ballToBeFound", "red")
				return 'ball not already detected'
		elif (room == "living room"):
			if(green_ball.detected_flag == True):
				ball_room = green_ball
				rospy.set_param("room", "")
			else:
				rospy.set_param("ballToBeFound", "green")
				return 'ball not already detected'
		elif (room == "kitchen"):
			if(yellow_ball.detected_flag == True):
				ball_room = yellow_ball
				rospy.set_param("room", "")
			else:
				rospy.set_param("ballToBeFound", "yellow")
				return 'ball not already detected'
		elif (room == "bathroom"):
			if(magenta_ball.detected_flag == True):
				ball_room = magenta_ball
				rospy.set_param("room", "")
			else:
				rospy.set_param("ballToBeFound", "magenta")
				return 'ball not already detected'
		elif (room == "bedroom"):
			if(black_ball.detected_flag == True):
				ball_room = black_ball
				rospy.set_param("room", "")
			else:
				rospy.set_param("ballToBeFound", "black")
				return 'ball not already detected'
		
		print ("Going to: ", room)
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = ball_room.location.x
		goal.target_pose.pose.position.y = ball_room.location.y
		goal.target_pose.pose.orientation.w = 1.0
		result = movebase_client(goal)
		if result:
			print ('I have reached ', room)	
		rospy.set_param("playflag", 0)
	rospy.set_param("playflag", 0)
	return 'after finishing play time'

class Find(smach.State):
    """
	In the PLAY state when the desired location is not yet explored, the robot switches to the FIND state. In this state, first the explore_lite package is activated for a certain time to explore and detect the required ball.

	While the time is running, detectBall() funtion is called. After returning from the detectBall() function we check if the required ball is found or not. If the ball is found, the state switches to PLAY state. If the ball is not found, it keep's on calling the detectBall() function till the timer expires. After the timer expires, the explore_lite package is shut and the robot switches to the PLAY state.   
   
    """
	
    def __init__(self): 
        
        smach.State.__init__(self, 
                             outcomes=['required ball found', 
				       'required ball not found'])
        
    def execute(self, userdata):
       
	global ballFound, child
        rospy.loginfo('Executing state FIND')
        cancel_msg = GoalID()
	ballFound = Ball()
	child = subprocess.Popen(["roslaunch","exp_assignment3","explore.launch"])
	rospy.set_param("expLiteFlag", 1)
	ballToBeFound = rospy.get_param("ballToBeFound")
	
	timer = time.clock()
	while(time.clock() - timer < 240):
		detectBall()
		if (ballToBeFound == 'blue'):
		   ballFound = blue_ball
		elif (ballToBeFound == 'red'):
		   ballFound = red_ball
		elif (ballToBeFound == 'green'):
		   ballFound = green_ball
		elif (ballToBeFound == 'yellow'):
		   ballFound = yellow_ball
		elif (ballToBeFound == 'magenta'):
		   ballFound = magenta_ball
		elif (ballToBeFound == 'black'):
	   	   ballFound = black_ball
		if (ballFound.detected_flag == True):
			child.send_signal(signal.SIGINT)
			pub_cancel.publish(cancel_msg)
			rospy.set_param("expLiteFlag", 0) 
			rospy.set_param("ballToBeFound", "")
			return 'required ball found'
	child.send_signal(signal.SIGINT)
	pub_cancel.publish(cancel_msg)
	rospy.set_param("expLiteFlag", 0)
	return 'required ball not found'
        

def main():
    """
    This is a State Machine for a Robot Dog name wheely.
    The main function initializes the ros node, subscribes to the topic: robot's odometry topic ("/odom"), laser scan data (/scan), and the robot's camera image topic ("camera1/image_raw/compressed"), and creates a publisher for the robot's velocity commands on the topic ("/cmd_vel"), a publisher for  cancelling the goal prvided to the move_base server ("/move_base/cancel"). 
	
    This function also intializes the "detectBallFlag", "expLiteFlag", "playflag", "nearhuman", "room", "ballToBeFound" parameters in the ROS Parameter Server.
    This function also defines the state machines and adds four states/behaviors to the container.
    """

    global pub_cmd_vel, pub_cancel
    rospy.set_param("expLiteFlag", 0)
    rospy.set_param("playflag", 0)
    rospy.set_param("nearHuman", 0)
    rospy.set_param("room", "")
    rospy.set_param("ballToBeFound", "")
    sub_camera = rospy.Subscriber("/camera1/image_raw/compressed",
                                           CompressedImage, imageCallback,  queue_size=1)
    sub_odom = rospy.Subscriber("/odom", Odometry, robotPos)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    pub_cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

    rospy.init_node('Assignment_3_State_Machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'after finishing normal behavior':'SLEEP',
					    'speech command:play': 'PLAY'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'after finishing sleep time':'NORMAL'})
	smach.StateMachine.add('PLAY', Play(), 
                               transitions={'after finishing play time':'NORMAL',
					    'ball not already detected': 'FIND'})
	smach.StateMachine.add('FIND', Find(), 
                               transitions={'required ball found':'PLAY',
					    'required ball not found':'PLAY'})


    # Create and start the introspection server for visualization. We 
    # can visualize this is smach viewer
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine 
    smach_ros.set_preempt_handler(sm)
    
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application 
    
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
