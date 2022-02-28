#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/******** Mechanical Spasefication ********/
#define Wheel_Diameter           3.25
#define Robot_Width              0.30
#define Encoder_TPR		(13.0*30.0)

/******** Motion Variables ********/
#define M_PI 			(3.141592653589793238)
double dl = 0.0 ;
double dr = 0.0 ; 
double dc = 0.0 ; 
double dt = 0.0 ; 

/******** Distance Variables ********/
double x = 0.0 ; 
double y = 0.0 ; 
double th = 0.0 ; 
double newTH = 0.0 ;
double avgTH = 0.0 ; 

ros::Publisher *pubPtr;

void ArduinoCB(const geometry_msgs::Vector3& msgIn) 
{	
	nav_msgs::Odometry msgOut;
	
	// update motion variables 
	dl = (double)msgIn.x / 100 ; 
	dr = (double)msgIn.y / 100 ; 
	
	dc = ((dr + dl) / 2) ;

	newTH = th + ((dr - dl) / Robot_Width) ;
	newTH = atan2(sin(newTH), cos(newTH));
	avgTH = atan2(sin(newTH)+sin(th), cos(newTH)+cos(th)); 

	// update disatnace variables 
	x += (dc * cos(avgTH)) ; 
	y += (dc * sin(avgTH)) ; 
	th = newTH ; 

	msgOut.pose.pose.position.x = x ; 
	msgOut.pose.pose.position.y = y ; 
	msgOut.pose.pose.position.z = 0.0 ; 
	msgOut.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th) ;

	for(int i = 0; i < 6; i++)	msgOut.pose.covariance[7*i] = 0.001 ;

	//Set the time stamp
	msgOut.header.stamp = ros::Time::now();
	
	//Publish the message
	pubPtr->publish(msgOut);
}

int main(int argc , char** argv) 
{
	/******** ROS Parameters ********/
	ros::init (argc , argv , "odom_translator") ;
	ros::NodeHandle nh ;
	pubPtr = new ros::Publisher(nh.advertise<nav_msgs::Odometry>("/odom" ,100));
	ros::Subscriber sub = nh.subscribe("/wheel_odom" , 1000 ,&ArduinoCB) ;
	ros::spin() ; 

	delete pubPtr;
}
