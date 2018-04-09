#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <func_header.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <sensor_msgs/LaserScan.h>
using namespace cv;
using namespace cv::xfeatures2d;

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state; // Current state
bool bumperflag = 0, cliff_flag = 0;

/*
0: Neutral, Follow me state 
1: Surprised, bumper
2: Nervous -> Scared, image
3: Excited, cliff
4: Sad, lost people
*/

//laser variables
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
double dist = 11;
double pi = 3.1416;
double yawStart = 0;
double d = 0.52;


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

	// to display image!
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	vel.angular.z = angular;
	vel.linear.x = linear;

	// sc.playWave(path_to_sounds + "sound.wav");
	// ros::Duration(0.5).sleep();

	//images
	Size size(1366,768);//the dst image size,e.g.100x100
	cv::Mat fear_image, excited_image, sad_image, surprised_image, neutral_image, nervous_image;
	
	cv::Mat fear = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobScared.png");
    resize(fear, fear_image, size);//resize image
	
	cv::Mat excited = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobExcited.jpg");
	resize(excited, excited_image, size);//resize image
	
	cv::Mat sad = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobSad.jpg");
	resize(sad, sad_image, size);//resize image
	
	cv::Mat surprised = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobSurprised.jpg");
	resize(surprised, surprised_image, size);//resize image
	
	cv::Mat neutral = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobNeutral.jpg");
	resize(neutral, neutral_image, size);//resize image
	
	cv::Mat nervous = imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/images/SpongebobNervous.jpg");
	resize(nervous, nervous_image, size);//resize image
	
	cv::Mat imgs_track;	// The wanted image we want to match to

	//imageTransporter imgTransport("camera/image/", sensor_msgs::image_encodings::BGR8); // For Webcam
	imageTransporter imgTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //For Kinect UNCOMMENT
	
	// Parameter Initialization
	int foundPic = 0;
	int distance_threshold = 1.8; //m
	
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................

		
		ROS_INFO("VELOCITY: %f\n", follow_cmd.linear.x);
		vel_pub.publish(follow_cmd);
		ros::spinOnce();
		
		// Reset current emotion to neutral every timestep
		cv::imshow("CurrentEmotion", neutral_image);
		cv::waitKey(30);

		/*
		0: Neutral, Follow me state 
		1: Surprised, bumper
		2: Nervous -> Scared, image
		3: Excited, cliff
		4: Sad, lost people
		*/
		
		printf("dist: %f \n",dist);
		if(follow_cmd.linear.x < 0.1 && follow_cmd.angular.z < 0.1 && dist > distance_threshold){//robot almost not moving and distance is greater than distance threshold
			 world_state = 4; // sad, lost
		}

		if (bumperflag == 1) {
			world_state = 1; // surprised, bumper
			bumperflag = 0;
		}
		else if (cliff_flag == 1) {
			world_state = 3; // excited. cliff
			cliff_flag = 0;
		}

		printf("World State %d \n", world_state);
		if(world_state == 0){
			// Neutral state
			// may add condition to only check image when robot stay still
			if(follow_cmd.linear.x < 0.1 && follow_cmd.angular.z < 0.1){
				foundPic = findPic(imgTransport, imgs_track, 1); //0: not found, 1: found
			}
			
			if(foundPic == 1){ // Check if we detect the Spongebob wanted poster
				world_state = 2;// Scared, image
			}
		}else if(world_state == 1){
			// Surprised, bumper
			// 4 sec soundtrack
			cv::imshow("CurrentEmotion", surprised_image);
			cv::waitKey(30);

			sleep(0.5);
			sc.playWave(path_to_sounds + "SpongeBobSurprised.wav");
			sleep(0.5);

			vel.angular.z = 0.0;
            vel.linear.x = -0.5;
			vel_pub.publish(vel);
			sleep(1.0);
			
			world_state = 0;
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			sleep(3);
			
			bumperflag = 0;
			world_state = 0;// reset world state
		}else if(world_state == 2){
			// Nervous -> Scared, image
			// 10 sec soundtrack
			cv::imshow("CurrentEmotion", nervous_image);
			cv::waitKey(10);

			// sleep(0.5);
			vel.angular.z = 0.0;
			sc.playWave(path_to_sounds + "SpongeBobScared.wav");
			vel.linear.x = -0.2;
			vel_pub.publish(vel);
			sleep(3.0);

			cv::imshow("CurrentEmotion", fear_image);
			cv::waitKey(10);
			// fear backoff
            vel.linear.x = -1;
			vel_pub.publish(vel);
			sleep(6);
			
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			sleep(1);
			foundPic = 0;
			world_state = 0;
		}
		else if(world_state == 3){
			// Excited, cliff
			// 5 sec soundtrack
			cv::imshow("CurrentEmotion", excited_image);
			cv::waitKey(30);

			sleep(0.5);
			sc.playWave(path_to_sounds + "SpongeBobExcited.wav");
			vel.angular.z = 0.0;
            vel.linear.x = 0.6;
			vel_pub.publish(vel);
			sleep(4);
			
			world_state = 0;
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			sleep(1.0);

			world_state = 0;
		}
		else if(world_state == 4){
			// Sad, lost people
			// 8 sec soundtrack
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
		ros::spinOnce();
	}

	return 0;
}