#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/Twist.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"

#include "localization_nao_pkg/Points.h"
#include "localization_nao_pkg/PPVector.h"

using namespace cv;
using namespace std;
	
vector<double> HeadPitchVector;

localization_nao_pkg::PPVector point_to_point_vector;

double PI = 3.1415926535897;
double speed=10;
double angle=30;

int cnt=40;

void StraightMover();

//--------------------------------------------------------------------//
///////////////-----------------------------------------////////////////
//--------------------------------------------------------------------//


class MoverStarter
{
	//Create a node handler it is reference assigned to a new node
	ros::NodeHandle n;
	ros::Publisher joint_pub;
	ros::Publisher chatter_pub;
	ros::Subscriber line_sub;
	ros::Subscriber joint_sub;

	public:
	  	MoverStarter()
	  	{

	  		//joint_sub = n.subscribe("/joint_states", 10, &MoverStarter::StraightMover, this);

	        /*Create a publisher for a topic '/cmd_vel' that will send a geometry message
			The second parameter to advertise() is the size of the message queue
			used for publishing messages.  If messages are published more quickly
			than we can send them, the number here specifies how many messages to
			buffer up before throwing some away.*/
	        //joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states",10);

	        chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",5);

			//subscribe to a given topic
			//chatterCallBack: is the name of the callback function that will be executed
			//each time a message is recieved
	  		line_sub = n.subscribe("point_to_point_line_with_factor", 10, &MoverStarter::chatterCallback, this);

		}


		void chatterCallback(const localization_nao_pkg::PPVector& msg)
		{

			localization_nao_pkg::Points pts;

			point_to_point_vector.pp_vector.clear();

			for(size_t i=0;i<msg.pp_vector.size();i++)
			{

				pts.x1=msg.pp_vector[i].x1;
				pts.y1=msg.pp_vector[i].y1;
				pts.x2=msg.pp_vector[i].x2;
				pts.y2=msg.pp_vector[i].y2;
				pts.factor=msg.pp_vector[i].factor;
				point_to_point_vector.pp_vector.push_back(pts);

			}

			StraightMover();
		}

		void StraightMover()
		{

			double curr_angle=0;

			geometry_msgs::Twist g_msg;
			g_msg.linear.x=0;
			g_msg.linear.y=0;
			g_msg.linear.z=0;
			g_msg.angular.x=0;
			g_msg.angular.y=0;

			double angular_speed = speed*2*PI/360;

		  	double relative_angle = angle*2*PI/360;//degrees to rad

		  	ros::Time::init();
		  	double t0 = ros::Time::now().toSec();
		  	double t1;
		  	//ROS_INFO("t0 = [%f]\n", t0);
		  		
		  	if(point_to_point_vector.pp_vector[0].x1<20 && point_to_point_vector.pp_vector[0].y1>200 && point_to_point_vector.pp_vector[0].factor<0)
		  	{

		  		//we turn to the right
		  		g_msg.angular.z=-angular_speed;//degrees/sec to rad/sec

		  		//trying to move little by little until we reach the angle we want
				while(curr_angle<relative_angle)
				{
				  	chatter_pub.publish(g_msg);
				  	t1 = ros::Time::now().toSec();
				  	curr_angle=angular_speed*(t1-t0);
				}

				//cout << "Speed in rads/sec = " << fixed << angular_speed << endl;
				//cout << "Angle in rads/sec = " << fixed << relative_angle << endl<< endl;
				//cout << "t0 = " << fixed << t0 << endl;
				//cout << "t1 = " << fixed << t1 << endl;
				//cout << "t1-t0 = " << fixed << t1-t0 << endl;

				//now we force to stop moving
				for(int i=0;i<10;i++)
				{
				  	g_msg.angular.z=0;
					chatter_pub.publish(g_msg);
				}
			
		  	}
		  	else if(point_to_point_vector.pp_vector[0].x2>300 && point_to_point_vector.pp_vector[0].y2>200 && point_to_point_vector.pp_vector[0].factor>0)
		  	{

		  		//we turn to the left
		  		g_msg.angular.z=angular_speed;//degrees/sec to rad/sec

		  		//trying to move little by little until we reach the angle we want
				while(curr_angle<relative_angle)
				{
				  	chatter_pub.publish(g_msg);
				  	t1 = ros::Time::now().toSec();
				  	curr_angle=angular_speed*(t1-t0);
				}

				//cout << "Speed in rads/sec = " << fixed << angular_speed << endl;
				//cout << "Angle in rads/sec = " << fixed << relative_angle << endl<< endl;
				//cout << "t0 = " << fixed << t0 << endl;
				//cout << "t1 = " << fixed << t1 << endl;
				//cout << "t1-t0 = " << fixed << t1-t0 << endl;

				//now we force to stop moving
				for(int i=0;i<10;i++)
				{
				  	g_msg.angular.z=0;
					chatter_pub.publish(g_msg);
				}

		  	}
		  	else
		  	{

		  		//if exists just to make the robot stop moving for a little
		  		if(cnt>=40)
		  		{
			  		g_msg.angular.z=0;//degrees/sec to rad/sec
			  		g_msg.linear.x=speed*PI/360;//we move front

			  		//trying to move little by little until we reach the angle we want
					while(curr_angle<relative_angle)
					{
					  	chatter_pub.publish(g_msg);
					  	t1 = ros::Time::now().toSec();
					  	curr_angle=angular_speed*(t1-t0);
					}

					//cout << "Speed in rads/sec = " << fixed << angular_speed << endl;
					//cout << "Angle in rads/sec = " << fixed << relative_angle << endl<< endl;
					//cout << "t0 = " << fixed << t0 << endl;
					//cout << "t1 = " << fixed << t1 << endl;
					//cout << "t1-t0 = " << fixed << t1-t0 << endl;

					//now we force to stop moving
					for(int i=0;i<10;i++)
					{
					  	g_msg.angular.x=0;
						chatter_pub.publish(g_msg);
					}

					cnt=0;

				}
				else
				{

					for(int i=0;i<10;i++)
					{
					  	g_msg.linear.x=0;
						g_msg.linear.y=0;
						g_msg.linear.z=0;
						g_msg.angular.x=0;
						g_msg.angular.y=0;
					  	g_msg.angular.z=0;
						chatter_pub.publish(g_msg);
					}

				}

				cnt++;

		  	}

		  	cout << "DONE THE MOVE!!!!!!!!!!" << endl << endl;


		}


	
};




int main(int argc, char **argv)
{

	ROS_INFO("Started mover node");

	ros::init(argc, argv, "mover_node");// Initiate a new ROS node named listener
	MoverStarter ms;
	ros::spin();// Enter a loop, pumping callbacks
	return 0;

}