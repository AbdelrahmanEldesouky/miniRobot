#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/******** Mechanical Spasefication ********/
#define Wheel_Diameter           0.0325
#define Robot_Width              0.41
#define Encoder_TPR		(13.0*30.0)

/******** Motion Variables ********/
#define M_PI 			(3.141592653589793238)
double dl = 0.0 ;
double dr = 0.0 ; 
double dc = 0.0 ; 
double dt = 0.0 ; 

/******** Ticks Variables ********/
double Right_OldTicks = 0.0 ; 
double Right_NewTicks = 0.0 ;
double Left_OldTicks = 0.0 ; 
double Left_NewTicks = 0.0 ; 

/******** Distance Variables ********/
double x = 0.0 ; 
double y = 0.0 ; 
double th = 0.0 ; 
double newTH = 0.0 ;
double avgTH = 0.0 ;

double vx = 0.0 ; 
double vy = 0.0 ; 
double vth = 0.0 ;

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
}

int main(int argc , char** argv) 
{
	/******** ROS Parameters ********/
	ros::init (argc , argv , "odom_basic") ;
	ros::NodeHandle nh ;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
	tf::TransformBroadcaster trans_broadcaster;
	ros::Subscriber sub = nh.subscribe("/wheel_odom" , 1000 ,&ArduinoCB) ;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(10.0);

	//-----------------------------------While loop--------------------------------

	while(nh.ok())
	{
		ros::spinOnce(); // check for incoming messages

		current_time = ros::Time::now();

		//---odom calculation------
		dt = (current_time - last_time).toSec();

		vx += (dc * cos(avgTH)) / dt ; 
		vy += (dc * sin(avgTH)) / dt ; 
		vth += ((dr - dl) / Robot_Width) /dt ;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first,---------- TRANSFORMATION broadcasting over tf
		geometry_msgs::TransformStamped odom_trans;

		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		trans_broadcaster.sendTransform(odom_trans);

		//next, ------------------ODOMETRY--------------- message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		for(int i = 0; i < 6; i++)	odom.pose.covariance[7*i] = 0.001 ;

		//set the velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//send THE ODOM message
    		odom_pub.publish(odom);

		last_time = current_time;

		r.sleep();

	}
}
