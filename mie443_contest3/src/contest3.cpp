#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <func_header.h>
#include <kobuki_msgs/BumperEvent.h>
using namespace cv;
using namespace cv::xfeatures2d;

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state; // Current state
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
// int world_state_prev; // Key track of previous state
/*
0: Following person (Play Pink Panther song)
1: Bumps into obstacle (Fear)
2: Lifted up 
*/

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
	printf("Reached Callback\n");
	if(msg.bumper == 0 || msg.bumper == 1 || msg.bumper == 2){
		world_state = 1;
		if(msg.bumper == 0){
			bumperLeft = !bumperLeft;
			printf("CHANGE LEFT\n");
		}else if(msg.bumper == 1){
			bumperCenter = !bumperCenter;
			printf("CHANGE CENTER\n");
		}else if(msg.bumper == 2){
			bumperRight = !bumperRight;
			printf("CHANGE RIGHT\n");
		}
	}
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("/mobile_base/events/bumper", 10, &bumperCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	// sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	sleep(0.5);
	// sc.playWave(path_to_sounds + "Spongebob angry.wav");
	sleep(1.0);

	//images
	cv::Mat A = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/fear.png");
	cv::Mat B = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/Worrying.png");
	cv::Mat Wanted_image = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/wanted.png");

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
		if(follow_cmd.linear.x < 0.1 && follow_cmd.angular.z < 0.1){//robot almost not moving
			world_state = 2;
		}
		printf("is it here?0\n");
		if(world_state == 0){
			vel_pub.publish(follow_cmd);
		}else if(world_state == 1){
			cv::imshow("CurrentEmotion", A);
			cv::waitKey(30);

			sleep(0.5);
			sc.playWave(path_to_sounds + "Spongebob angry.wav");
			sleep(1.0);
			world_state = 0;
		}else if(world_state == 2){
			printf("is it here?1\n");
			foundPic = findPic(imgTransport, imgs_track, 1); //0: not found, 1: found
			printf("is it here?2\n");
			if (foundPic == 0){
				printf("is it here?3\n");
				cv::imshow("CurrentEmotion", B);
				cv::waitKey(30);

				sleep(0.5);
				sc.playWave(path_to_sounds + "Spongebob crying.wav");
				sleep(1.0);
				printf("is it here?3.4\n");
			}else{//find the match
				printf("is it here?4\n");
				//fear!!!
				cv::imshow("CurrentEmotion", Wanted_image);
				cv::waitKey(30);

				sleep(0.5);
				sc.playWave(path_to_sounds + "Spongebob angry.wav");
				sleep(1.0);
				printf("is it here?24.5\n");
			}
			



			world_state = 0;
		}

		printf("World State: %d\n", world_state);
	}

	return 0;
}
