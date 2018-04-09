#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <func_header.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Bool.h"
using namespace cv;
using namespace cv::xfeatures2d;

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state; // Current state
bool bumperflag = 0, cliff_flag = 0;
// int world_state_prev; // Key track of previous state
/*
0: Following person (Play Pink Panther song)
1: Bumps into obstacle (Fear)
2: Lifted up 
*/
//laser variables
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
double dist = 11;
double pi = 3.1416;
double yawStart = 0;
double d = 0.52;
bool robot_stuck = false;


void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
	printf("Reached Callback\n");
	if(msg.bumper == 0 || msg.bumper == 1 || msg.bumper == 2){
		bumperflag = 1;
	}
}

void cliffCB(const kobuki_msgs::CliffEvent msg) {
	printf("Reached Cliff\n");
	if (msg.state == 1){
		cliff_flag = 1;
	}else{
		cliff_flag = 0;
	}
}

void stuckCB(const std_msgs::Bool msg){
	if(!msg.data){
		robot_stuck = false;
	}else{
		robot_stuck = true;
	}
}

void scanProfile (const sensor_msgs::LaserScan::ConstPtr& msg){ 
	int i;
	double min_angle = msg->angle_min + pi/2;	
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment; // Number of range indices in sensor field of view
	dist = 11; // In centimetres
	
	for (i = 0; i < laserSize; i++) { // loop through each laser scan index
		if(abs(msg->ranges[i] / tan(min_angle + i*msg->angle_increment)) <= d ){
			// laser ranges are within the left square
			if(msg->ranges[i] < dist) {
				dist = msg->ranges[i];
			}
		}
	}

	if(dist == 11){
		dist = 0;
	}
}


//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	ros::Subscriber laser_sub = nh.subscribe("/scan", 10, &scanProfile);
	

    if(robot_stuck){
		printf("Robot is stuck\n");
	}
    geometry_msgs::Twist vel;

	//publisher
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
	
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("/mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff = nh.subscribe("/mobile_base/events/cliff", 10, &cliffCB);
	ros::Subscriber stuck_sub = nh.subscribe("/turtlebot_follower/stuck", 10, &stuckCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	vel.angular.z = angular;
	vel.linear.x = linear;

	// sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	sleep(0.5);
	// sc.playWave(path_to_sounds + "Spongebob angry.wav");
	sleep(1.0);

	//images
	cv::Mat fear_image, excited_image, sad_image, surprised_image, neutral_image, nervous_image;
	cv::Mat fear = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobScared.png");
	Size size(1366,768);//the dst image size,e.g.100x100
    resize(fear, fear_image, size);//resize image
	cv::Mat excited = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobExcited.jpg");
	resize(excited, excited_image, size);//resize image
	cv::Mat sad = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobSad.png");
	resize(sad, sad_image, size);//resize image
	cv::Mat surprised = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobSurprised.jpg");
	resize(surprised, surprised_image, size);//resize image
	cv::Mat neutral = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobNeutral.png");
	resize(neutral, neutral_image, size);//resize image
	cv::Mat nervous = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobNervous.jpg");
	resize(nervous, nervous_image, size);//resize image
	
	
	

	cv::Mat imgs_track;	// The wanted image we want to match to

	//imageTransporter imgTransport("camera/image/", sensor_msgs::image_encodings::BGR8); // For Webcam
	imageTransporter imgTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //For Kinect UNCOMMENT
	
	int foundPic = 0;

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................
		
		// condition: change world_state

		//need two more conditions: 1: bumper, 2: lifting sensor
		ROS_INFO("VELOCITY: %f\n", follow_cmd.linear.x);
		vel_pub.publish(follow_cmd);

		cv::imshow("CurrentEmotion", neutral_image);
		cv::waitKey(30);

		printf("dist: %f \n",dist);
		if(follow_cmd.linear.x < 0.1 && follow_cmd.angular.z < 0.1 && dist > 1.5){//robot almost not moving
			 world_state = 4;
			 printf("dist: %f \n",dist);
		}

		
		if (bumperflag == 1) {
			world_state = 1;
		}
		else if (cliff_flag == 1) {
			world_state = 3;
		}

		printf("World State %d \n", world_state);
		if(world_state == 0){ // Default state of just following people
			foundPic = findPic(imgTransport, imgs_track, 1); //0: not found, 1: found
			if(foundPic == 1){ // Check if we detect the Spongebob wanted poster
				world_state = 2;
			}
		}else if(world_state == 1){
			//bumper
			cv::imshow("CurrentEmotion", surprised_image);
			cv::waitKey(30);

			sleep(0.5);
			sc.playWave(path_to_sounds + "SpongeBobSurprised.wav");
			sleep(1.0);

			vel.angular.z = 0.0;
            vel.linear.x = -1.5;
			vel_pub.publish(vel);
			sleep(1);
			
			world_state = 0;
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			sleep(3.5);
			
			bumperflag = 0;
			world_state = 0;

		}else if(world_state == 2){
			//image fear
			cv::imshow("CurrentEmotion", nervous_image);
			cv::waitKey(10);

			// sleep(0.5);
			sc.playWave(path_to_sounds + "SpongeBobScared.wav");
			sleep(4.0);
			cv::imshow("CurrentEmotion", fear_image);
			cv::waitKey(10);
			// fear backoff
			vel.angular.z = 0.0;
            vel.linear.x = -0.5;
			vel_pub.publish(vel);
			sleep(6);

			world_state = 0;
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			
		}
		else if(world_state == 3){
			//cliff
			cv::imshow("CurrentEmotion", excited_image);
			cv::waitKey(30);

			sleep(0.5);
			sc.playWave(path_to_sounds + "SpongeBobExcited.wav");
			vel.angular.z = 0.0;
            vel.linear.x = 0.6;
			vel_pub.publish(vel);
			sleep(10);
			
			world_state = 0;
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			sleep(1.0);

			world_state = 0;
		}
		else if(world_state == 4){
			//lost
			cv::imshow("CurrentEmotion", sad_image);
			cv::waitKey(30);

			sleep(0.5);
			sc.playWave(path_to_sounds + "SpongebobSad.wav");

			angular = 0.7;

			for (int i = 0; i < 3; i++){
				ros::spinOnce();
			
				vel.angular.z = angular;
				vel.linear.x = 0.0;
				vel_pub.publish(vel);
				sleep(1);
				vel.angular.z = 0.0;
				vel.linear.x = 0.0;
				vel_pub.publish(vel);
				sleep(0.5);
			
				ros::spinOnce();
			
				vel.angular.z = -angular;
				vel.linear.x = 0.0;
				vel_pub.publish(vel);
				sleep(1);
				vel.angular.z = 0.0;
				vel.linear.x = 0.0;
				vel_pub.publish(vel);
				sleep(0.5);
			}
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);

			//sleep(2.0);
			world_state = 0;
		}
	}

	return 0;
}