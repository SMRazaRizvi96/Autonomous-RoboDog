# ExpRobotics-Final Assignment

This is a finite state machine for a Mobile Robot being used inside a house.
The main idea of this assignment is to able a mobile robot to move around the environment, explore and map the environment, listen and perform actions to specific commands given by the user, and register the locations of different rooms with the help of different colored balls.
The four states of this mobile robot are explained below.

This is the 'Assignment 3 for the course 'Experimental Robotics' at The University of Genova.


#### How to Run:

    Go to the 'Master' branch and Clone the repository
    
    Copy the 'exp_assignment3' , ‘m-explore’, ‘planning’ and ‘slam_gmapping’ folder into the src folder of your ROS workspace
    
    Open the terminal and run 'catkin_make' inside your ROS workspace

    Install the Navigation Stack by running the following command:

	      sudo apt-get install ros-<ros_distro>-navigation

    Open the Terminal and go to the 'Scripts' folder of the package ‘exp_assignment3’

    Give running permissions to the state_machine with $ chmod +x assignment3.py

    Run the launch file $ roslaunch exp_assignment3 simulation.launch
    
    
************************** 

#### List of Packages:

‘exp_assignment3’: This is the package developed for the entire state machine of this robot.
‘planning’: This is a helping package used for the planning aspect.
‘slam_gmapping’: This is a helping package used for the SLAM purposes.
‘m-explore’: This the explore-lite package that is a helping package used for the autonomous navigation and exploration, and this packages uses the ‘planning’ and ‘slam_gmapping’ packages.

Now this README file will focus on defining the ‘exp_assignment3’ package developed specifically for this assignment. 

#### Gazebo Models:

The models used were defined in the XACRO and URDF formats.

##### Robot Model:

The Robot is a Dog modeled inside the robot2_laser.xacro file given in the urdf folder.
It has a base, with a laser scan camera, and a head with an rgb camera.
The Robot Dog has two continuous wheel at the back, and a caster wheel at the front.

The Robot uses a differential drive controller for the two wheels at the back.

##### Human Model:

The Human model is defined inside the human.urdf file given in the urdf folder.
It is a normal colorless human being sitting on a chair.

**************************  


#### Files List:

assignment3.py: This is the State Machine
Speak_node.cpp: This is the node provides a GUI to the user to give the ‘play’ command, and to tell the GoTo location to the Robot.

**************************    

#### ROS Parameters Used in the State Machine:

'expLiteflag': This parameter is a flag that is 1 if the explore-lite package is running, and 0 otherwise.
'playflag': This parameter is a flag that is 1 if the ‘play’ command is given by the user, and 0 otherwise.
‘nearHuman’: This parameter is a flag that is swtcked to 1 as soon as the robot reaches near the Human for receiving the GoTo location, while in the PLAY state.
‘room’: This parameter is used to save the GoTo location given by the user.
‘ballToBeFound’: This parameter saves the ball corresponding to the desired GoTo location.

**************************    

#### ROS Topics Used in the State Machine:

#####   Subscribed to:

'/odom' : Topic to get the Robot's odometry data

'/camera1/image_raw/compressed' : To get the Compressed RGB Camera Image

‘/scan’: To get the 180 degree laser scan data   
       
#####   Publishing to:

'/cmd_vel' : To publish velocity command to the Robot

'/move_base/cancel' : To cancel the goal given to the move_base server

**************************    

#### ROS Custom Messages Used:

#####   Ball.msg:

This message has the following attributes:
bool detected_flag: This flag is turned to 1 if the corresponding instance of the ball is detected
geometry_msgs/Point location: This attribute saves the location of the ball

The following instances of this message are created corresponding to the different balls

black_ball = Ball()
red_ball = Ball()
yellow_ball = Ball()
green_ball = Ball()
blue_ball = Ball()
magenta_ball = Ball()

**************************    

#### Implementation Details of the State Machine:

In this Assignment, we have developed a Finite State Machine for a Mobile Robot that has four states:

1. NORMAL
2. SLEEP
3. PLAY
4. FIND

![alt text](https://github.com/SMRazaRizvi96/ExpRobotics-FinalAssignment/blob/master/Finite_State_Machine.png)

##### 1. NORMAL:

In this state, the robot moves on random locations for 4 minutes.
For this purpose, the ‘explore-lite’ package is used. The explore-lite package, with the help of ‘planning’ package and the ‘slam_gmapping’ package, generates frontiers in the environment that corresponds to the un-explored, disconnected regions of the environment. Using these frontiers, it generates a coordinate on the frontier where the robot should go to explore the environment, and send this coordinate to the move_base server on the Navigation stack, which takes care of generating the velocity commands. During all this time, the robot also implements SLAM.

In the NORMAL state, while the robot is moving, if the robot detects a colored ball, the ‘explore-lite’ package is closed, and the robot goes near the ball. The robot then saves its odometry location as the location of the detected ball, and hence of the corresponding room, and turns the corresponding detected_flag attribute of the ball to 1. For this functionality, it goes into a function, ‘detectBall()’.

##### 2. SLEEP:
In this state, the robot goes to the coordinate (0,0) and stays there for 5 to 10 seconds.
For this purpose, a goal with x and y coordinates set to 0, is sent to the move_base server, using the movebase_client() function.

##### 3. PLAY:
In the PLAY state, the robot has to go to the room given by the user and repeat this until the timer of 5 minutes runs out.
For this purpose, the robot first goes near the Human by sending the coordinate (-5, 8), to the move_base server using the movebase_client() function, and waits for a GoTo location.
If the coordinate of room given by the user is known, the corresponding coordinate is send to the move_base server, and the robot goes to the coordinate and then comes back to the human for the next GoTo location.
If the coordinate of room given by the user is unknown, the robot switches to the FIND state to find the  ball corresponding to the desired room.
If within the FIND state timer the robot is able to find the corresponding ball, the robot goes to the location, and then repeats the PLAY state.
If within the FIND state timer the robot is not able to find the corresponding ball, the robot comes back to the human and asks for the next GoTo location.

##### 4. FIND:
In the FIND state, the robot has to find the ball corresponding to the desired room until 4 minutes.
For this purpose, first the explore-lite package is launched to move the robot and explore the environment autonomously,  while also using the detectBall() function to detect any new ball in the environment. As soon as the ball corresponding to the desired room is detected, the explore-lite package is stopped and the robot goes near the ball. The robot then saves its odometry location as the location of the detected ball, and hence of the corresponding room, and turns the corresponding detected_flag attribute of the ball to 1.
The robot switches back to the PLAY state if the corresponding ball is detected, or if the timer runs out.

**************************

#### Functions Used in the State Machine:

##### robotPos():
This function is a callback for the robot's odometry topic "/odom".
This function extracts the x and y coordinate of the Robot, and saves them in the global robot_x and robot_y variables.


##### clbk_laser():
This function is a callback for the laser scan topic "/scan".
This function divides the 720 samples of the entire 180 degrees into two regions, ‘front_right’, and ‘front_left’, and calculates the minimum values of the laser scan, inside both the regions. These values are later on used for obstacle detection and avoidance.

'front_right': min(min(msg.ranges[180:360]), 10)
'front_left': min(min(msg.ranges[361:540]), 10)

![alt text](https://github.com/SMRazaRizvi96/ExpRobotics-FinalAssignment/blob/master/laserscan.png)


##### imageCallback(image)
This function is a callback of the compressed rgb image topic ‘/camera1/image_raw/compressed’.
This function converts the image into CV2 format, and saves it into a global variable.


##### detectBall():
This function uses Open CV to process the image received, and detects the contours of different colored balls in the image using different masks for each color. 
If any contour is found, it passes the radius and center of the detected contour, and also the detected ball object to the function track().


##### track(adius, center, detected_bal):
This function is responsible to move the robot towards the ball detected, and record the ball’s coordinate.
At first this functions cancels the goal published on the move_base server and shuts down the explore-lite package. After doing so, this function implements a control to move the robot towards the detected ball until the robot is sufficiently close to the ball. For this, the radius of the detected ball is checked and when it becomes greater than 90, the robot stops, and saves its own odometry coordinate as the coordinate of the ball, and hence of the corresponding room. The explore-lite package is started again once the coordinate is saved.
During this movement, the robot also has to avoid the obstacles so this functions checks the front_right and front_left regions laser scan values. If the minimum value in any of the two above mentioned regions becomes less than 0.5, the obstacle is assumed to be very near and the function stops the liear motion of the robot and rotates the robot in the opposite direction of the obstacle to avoid the obstacle. Once the minimum value in both the regions becomes greater than 0.5, the function continues to move the robot towards the detected ball.


##### movebase_client(goal):
This function serves as a move_base client. It publishes a goal to the move_base server and wait until the target has been reached.

**************************

#### Limitations:
	
1. Since the System is not using any voice commands, one of the limitations is to type the command rather than saying it.
2. Currently the robot could only move  only in 2D.
3. The Robot can only detect  specific balls only.
4. Once the entire environment is explored, the robot will not move in the NORMAL state.

**************************

#### Authors:

Syed Muhammad Raza Rizvi (S4853521): S4853521@STUDENTI.UNIGE.IT

Laiba Zahid (S4853477): S4853477@STUDENTI.UNIGE.IT

The algorithm was drafted and finalized after a discussion between the authors and was then implemented together.
