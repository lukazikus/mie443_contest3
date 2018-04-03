#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <func_header.h>
#include <math.h>

#include <eStop.h>
using namespace cv;
using namespace cv::xfeatures2d;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x;
float y;
float phi;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    	x = msg.pose.pose.position.x;
    	y = msg.pose.pose.position.y;
}

//-------------------------move robot function---------------
bool moveToGoal(float xGoal, float yGoal, float phiGoal){
	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
    	geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = phi.z;
	goal.target_pose.pose.orientation.w = phi.w;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}

int nearest_n (std::vector<std::vector<float> > path, int n){ //finds which x-y pair is closest and returns index
	
	float length = 0, min_l = 0;
	int i, min_index = 0;

	for(i=0; i<n; i++){
		length = sqrt(pow((path[i][0]-path[n][0]),2) + pow((path[i][1]-path[n][1]),2)); //sqrt((x-x_last)^2 + (y-y_last)^2);
		if(i==0){
			min_l = length;
			min_index = i;
		}
		if(length<min_l){
			min_l = length;
			min_index = i;
		}
	}
	return min_index;
}

std::vector<std::vector<float> > find_path (std::vector<std::vector<float> > coordinates){ //takes coordinate list and creates a new, ordered list
	
	int i,j, next_index;
	float temp_x, temp_y, temp_z;//make copy of the coordinate array to modify

	std::vector<std::vector<float> > path (6, std::vector<float>(3,0));
	path[0][0] = 1;

	for (i=0; i<6; i++){
		for(j=0; j<3; j++){
			path[i][j] = coordinates[i][j];
		}
	}

	for(i=5; i>0; i--){
		next_index = nearest_n(path, i);//returns index of nearest neighbor to path(i)
		//now swap pairs to order them
		temp_x = path[i-1][0];
		temp_y = path[i-1][1];
		temp_z = path[i-1][2];
		path[i-1][0] = path[next_index][0];
		path[i-1][1] = path[next_index][1];
		path[i-1][2] = path[next_index][2];
		path[next_index][0] = temp_x;
		path[next_index][1] = temp_y;
		path[next_index][2] = temp_z;
	}
	return path;
}	

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
  	teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<cv::Mat> imgs_track;	// Three images we want to match to
	float BlockDist = 0.7;
	bool raisinRep, riceRep, roastRep; // Duplicate flags for each cereal type
	raisinRep = false;
	riceRep = false;
	roastRep = false;

	if(!init(coord, imgs_track)) return 0;

	for(int i = 0; i < coord.size(); ++i){
		coord[i][0] = round((coord[i][0] + BlockDist * cos(coord[i][2]))*1000)/1000;
		coord[i][1] = round((coord[i][1] + BlockDist * sin(coord[i][2]))*1000)/1000;
		if (coord[i][2] < 0) {
			coord[i][2] = coord[i][2] + 3.14;
		}
		else {
			coord[i][2] = coord[i][2] - 3.14;
		}
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
	}
	
	//imageTransporter imgTransport("camera/image/", sensor_msgs::image_encodings::BGR8); // For Webcam
	imageTransporter imgTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //For Kinect UNCOMMENT
	
	int foundPic = 0;

	while(ros::ok()){
		ROS_INFO("HI\n");
		ros::spinOnce();
  	 	//.....**E-STOP DO NOT TOUCH**.......
   	 	eStop.block();
	 	//...................................

	 	//fill with your code
		
		//change address*******************************************************************************************
		Mat imgs_track;

		int i,j; //create coordinate array
		std::vector<std::vector<float> > coordinates(6, std::vector<float>(3,0));
		
		for(i=0; i<5; i++){
			for(j=0; j<3; j++){
				coordinates[i][j] = coord[i][j];
			}
		}
		coordinates[5][0] = x;	
		coordinates[5][1] = y; 
		coordinates[5][3] = phi;

		
		std::vector<std::vector<float> > path (6, std::vector<float>(3,0));
		path = find_path(coordinates);
		
		//REMOVE this line!!! Add this line for testing image processing purpose
		//continue;

		for(i=0; i<5; i++){

			for(j=0; j<3; j++){
				printf("%f ",path[i][j]);
			}
			printf ("\n");
		}
		
		std::vector<int> box_sequence(5);
		for (i = 0; i < 5; i++){
			box_sequence.at(i) = 0;
		}
		for(i=4; i>=0; i--){
			ros::spinOnce();
			printf("Going to location: %d\n", i);
			moveToGoal(path[i][0], path[i][1], path[i][2]); // Moves robot to next goal
			ros::spinOnce();
			foundPic = findPic(imgTransport, imgs_track, i); //1:raisin bran, 2: cinnimen roast crunch, 3: rice krispies, 4:blank image
			box_sequence.at(i) = foundPic;
		}
		for(i=4; i>=0; i--){
			if (box_sequence.at(i) == 1){
				if(!raisinRep){
					printf("Location %d: Raisin Bran\n", 5-i);
				}else{
					printf("Location %d: Raisin Bran (duplicate)\n", 5-i);
				}
				raisinRep = true;
			}else if (box_sequence.at(i) == 2){
				if(!roastRep){
					printf("Location %d: Cinnamon Toast Crunch\n", 5-i);
				}else{
					printf("Location %d: Cinnamon Toast Crunch (duplicate)\n", 5-i);
				}
				roastRep = true;
			}else if (box_sequence.at(i) == 3){
				if(!riceRep){
					printf("Location %d: Rice Krispies\n", 5-i);
				}else{
					printf("Location %d: Rice Krispies (duplicate)\n", 5-i);
				}
				riceRep = true;
			}else if (box_sequence.at(i) == 4){
				printf("Location %d: Blank Image\n", 5-i);
			}else{
				printf("Error\n");
			}
		}
		
		moveToGoal(coordinates[5][0], coordinates[5][1], coordinates[5][2]);
		break;
	}

	return 0;
}