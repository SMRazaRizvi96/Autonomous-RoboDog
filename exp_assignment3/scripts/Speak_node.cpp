#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{	
/** This node implents the GUI for giving the play command to the robot and 
  * the goto location. It does this by checking and setting some paramters using ROS 
  * parameter server.  The location is stored in paramter 'room' and 'playflag' 
  * parameter is set to 1 as soon as the user has given the play command. */
	
  
  ros::init(argc, argv, "Speak_node"); 
  ros::NodeHandle n;
  
  std_msgs::String msg, location;
  int s, check;  

 
  while (ros::ok())
  {
    n.getParam("playflag", s);
    if (s == 0)
    {
       cout << "Do you want to play with me? If yes, please write 'play' ";
       getline(cin, msg.data);
    }

    //ROS_INFO("%s", msg.data.c_str());

    if(msg.data == "play")
    {
    	n.setParam("playflag", 1);
    }

    n.getParam("nearHuman", check);

    if (check == 1)
    {    
	n.setParam("nearHuman", 0);
	cout << "Let's Play! Where should I go?\nGoto: ";
	getline(cin, location.data);
	n.setParam("/room", location.data);
    }
  }

  return 0;
}
