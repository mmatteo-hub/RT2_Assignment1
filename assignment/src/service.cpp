/**
 * \file service.cpp
 * \brief Node to manage the correct drive mode according to the user choice.
 * \author Matteo Maragliano
 * \version	1.0
 * \date 27/02/2022
 * 
 * \details 
 * 
 * Subscriber to: <BR>
 * /move_base/feedback This is the topic to have always the status of the robot available for the use.
 * 
 * /move_base/goal This topic allows the program have always the goal status available to perform the right flow of the program.
 * 
 * /cmd_vel This topic allows the program have always the robot's velocity available.
 * 
 * /scan This topic is used to take the laser scan values and compute the distance of the robot from an obstacle.
 * 
 * Publisher to: <BR>
 * /move_base/goal Topic used to publish the current goal inserted by the user.
 * 
 * /move_base/cancel Topic to delete the current goal used by the robot.
 * 
 * /cmd_vel Topic to publish the velocity the robot has to perform: used to stop the robot when deleting the goal or when there is an obstacle.
 * 
 * Services: <BR>
 * /service This is the service used to take from the UI node the user choice inserted.
 * 
 * Description:
 * This node is the service node which allows the program compute the different choices inserted by the user.
 * In particular the UI node sends the user choice through a service and the service node analizes the choice computing the different actions.
 * 
**/

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "assignment/Service.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "stdio.h"
#include "string.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "geometry_msgs/PointStamped.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

// size for the array
#define SIZE 144 ///< Size of the array

// define for the distance
#define th 0.5 ///< Distance threashold for obstacle

// define for the distance of the wall
#define wth 1 ///< Distance from the wall

// variable to determine the assistance during drive
bool assistDrive = false; ///< Global variable to manage the drive assistance activation.
//variable to allow the manual
bool manual = false; ///< Global variable to manage the manual drive activation.
// variable to know if there is a goal
bool G = false; ///< Global variable to determine the presence of a goal.

// define publishers:
// pub for the goal 
ros::Publisher pub; ///< Defining a publisher for the goal publication.
// pub to cancel the goal
ros::Publisher pubCancel; ///< Defining a publisher for the goal deletion.
// pub to publishthe velocity
ros::Publisher pubV; ///< Defining a publisher for the velocity publication.

// define a variable to publish
move_base_msgs::MoveBaseActionGoal pose; ///< Global variable to store the position of the robot.

// define a string to save the goal id
std::string goalID; ///< Global variable to store the goal ID of the robot.

// define a variable for reading fields of the goal to cancel
actionlib_msgs::GoalID goalToCancel; ///< Global variaible to store the ID of the goal that has to be deleted.

// define a variable to publish the new velocity
geometry_msgs::Twist n; ///< Global variable as message to pass data with a proper format.

// define variables to store the goal
float xG; ///< Global variable to store the x-coordinate of the position inserted by the user.
float yG; ///< Global variable to store the y-coordinate of the position inserted by the user.

/**
 * \brief Function to set the goal of the robot.
 * \param float X and Y coordinates for the goal.
 * 
 * \return
 * 
 * The function takes as input the x and y value inserted by the user and set them as the target goal
 * that the robot has to reach.
 * It enters into the field ...position.x/y to set them, then sets the frame_id as map and the value
 * of the quaternion module to 1 (all to 0 except w to 1).
 * After having completed all these actions it publish the goal and set the variable G (the one to establish
 * the presence of a goal) to true so that the robot can know there is a goal to be reached.
 * 
**/
// function to set the parameters to the right field of the variable to publish
void setPoseParams(float inX, float inY)
{
	// set the value (x y) to the x and y field of the variable pose
	pose.goal.target_pose.pose.position.x = inX;
	pose.goal.target_pose.pose.position.y = inY;
			
	// set the frame_id
	pose.goal.target_pose.header.frame_id = "map";
	
	// set the quaternion module equal to 1
	pose.goal.target_pose.pose.orientation.w = 1;
	
	// publish the target chosen
	pub.publish(pose);
	
	// set the goal flag
	G = true;
}

/**
 * \brief Function to delete the actual goal.
 * \param
 * 
 * \return
 * 
 * This function is used to delete the actual goal that was previously set.
 * In particular the function checks the presence of the goal through the global variable G
 * and if there is a goal it takes the goal ID and publish through the cancel publisher; 
 * otherwise if there is any goal the function advertises the user with a print on the screen.
 * 
**/
// function to cancel the goal by the user input
void cancelGoal()
{
	system("clear");
	// check the presence of a goal
	if(G)
	{
		// set the goal id to cancel equat to the actual goalID
		goalToCancel.id = goalID;
		// publish to cancel
		pubCancel.publish(goalToCancel);
		// set the flag to flase
		G = false;
		// print
		std::cout << "Goal cancelled.\n";
	}
	else
	{
		// print
		std::cout << "No goal set.\n";
	}
}

/**
 * \brief Function to take the status: goal ID.
 * \param msg message from which infos regarding the robot's position are taken
 * 
 * \return
 * 
 * The function is set to keep periodically updated the current goal set: 
 * in particular the goal is periodically taken into a proper variable.
 * Moreover, if there is a goal already set, checked through the variable G,
 * and if the robot is sufficiently near the position speficied by the user
 * input (x and y coordinates) then the goal is automatically deleted to avoid
 * having it set even if the robot is not moving, thus saving memory space.
 * 
**/
// function to take the status: in particular the actual goal id
void takeStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
	// set the goalID variable with the value of the actual goal id
	goalID = msg -> status.goal_id.id;
	
	// check the presence of a goal
	if(G)
	{
		// check the distance of both coordinates to see if the robot is near the goal
		if(abs(msg -> feedback.base_position.pose.position.x - xG) <= th && abs(msg -> feedback.base_position.pose.position.y - yG) <= th)
		{
			system("clear");
			// cancel goal
			cancelGoal();
			system("clear");
			// print
			std::cout << "Goal reached successfully\n";
		}
	}
}

/**
 * \brief Function to store the current goal the robot has to rach.
 * \param m message from which infos regarding the robot's position are taken
 * 
 * \return
 * 
 * The function is used to take the robot goal in order to make it available
 * to the program to perform particular actions: for example the check of
 * the distance to allow the program delete it, or also to state that a 
 * particular position (x,y) inserted is not reachable by the robot.
 * 
**/
// function to store the current goal of the robot
void currGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& m)
{
	// get x coordinate of the current goal
	xG = m -> goal.target_pose.pose.position.x;
	// get y coordinate of the current goal
	yG = m -> goal.target_pose.pose.position.y;
}

/**
 * \brief Function to take the current velocity of the robot
 * \param m message from which infos regarding the robot are taken
 * 
 * \return
 * 
 * The function is used to read the robot's velocity.
 * In particular the function checks if the manual modality of driving
 * is enabled, throigh the manual global velocity; after that it also checks
 * the presence of the driving assistance through a proper variable; at the end
 * it stores the actual velocity of the robot into two different variables in order
 * to have them available.
 * 
**/
// function to take the velocity: always available the current vel of the robot
void takeVel(const geometry_msgs::Twist::ConstPtr& m)
{	
	// check the presence of the manual drive
	if(!manual)
	{
		// do nothing
		return;
	}
	// check the presence of the driving assistance
	if(!assistDrive)
	{
		// publish the message
		pubV.publish(m);
		return;
	}
	
	// update the velocity value
	n.linear.x = m -> linear.x;
	n.angular.z = m -> angular.z;
}

/**
 * \brief Function to display infos about how to enter a valid position for the goal the robot has to reach.
 * \param
 * 
 * \return
 * 
 * The function is used to print the instruction that the user has to follow in order to publish
 * a valid position for the goal that the robot has to reach.
 * There is also a print about the coordinated system used in order to allow the user know how to
 * choose a position and how the robot locates it.
 * 
**/
// menu to display inside the switch
void menu()
{
	std::cout << "\n###################### INFOS ######################\n";
	std::cout << "Enter a position with the 'x y' format.\nUse a space for distinguishing the coordinates.\nUse a dot . for decimal coordinates.\n";
	std::cout << "\nCartesian axes are set positive as follow:\n";
	std::cout << "\t^\n\t|\n\t|\n\t|\n\t|\n\t°---------->\nO = (0 0)\n";
	std::cout << "###################################################\n";
	std::cout << "\nType here: ";
}

/**
 * \brief Function to compute the minimul valie among an array
 * \param a[] an array of double that contains values representing the distances
 * 
 * \return min_val the minimum value that is contained inside the array
 * 
 * The functon is used to compute the minimum value among an array with a certain dimension.
 * In particular, each array element represents the distance from an obstacle that is detected by the laser scan.
 * The robot is provided with 720 sensors, so the array will have that dimension (SIZE in the function); the
 * maximum distance that the robot can detect is 30 and this is the value that is set as the minimim inside the dist
 * variable of the function.
 * For each element inside of the array the function compares it with the dist variable and if it is lower that this one
 * then it is updated with the new minimum value. At the end of the execution the function returns the minimum value.
 * 
**/
// function to calculate the minimum distance among array values
double min_val(double a[])
{
	// setting dist as the maximum for the laser so that there cannot be errors
	double dist = 30;
	
	// test each element of the array
	for(int i = 0; i < SIZE; i++)
	{
		// check if the value is less than the distance
		if(a[i] < dist)
		{
			// update tge distance with the value
			dist = a[i];
		}
	}
	
	// return the distance found as minimum
	return dist;
}

/**
 * \brief Function to enable/disable or quit the manual drive
 * \param input char that represents the command the user has inserted
 * 
 * \return
 * 
 * The function is used to enable, disable or quit the manual drive for the robot.
 * In particular there is a switch that takes as parameter a character inserted by
 * the user.
 * First of all the global variable manual is set to true to allow the user drive the
 * robot with the keyboard; then depending on the value read it is changed the value of
 * the global variable assistDrive (true when enabled, false when disabled) in order to
 * allow the program having or not the assistance during the drive.
 * At the end, when the user inserts the quit command (q) the function sets to false all
 * the two global variables, sets also to 0 the linear and angular velocity of the robot
 * and publishes them to stop the robot moving.
 * 
**/
// manual drive
void manuallyDrive(char input)
{
	manual = true;
	switch(input)
	{
		case 'm':
			assistDrive = true;
			break;
		
		case 'n':
			assistDrive = false;
			break;
			
		case 'q':
			manual = false;
			assistDrive = false;
			n.linear.x = 0;
			n.angular.z = 0;
			
			pubV.publish(n);
			
			break;
	}
}

/**
 * \brief Function to assit the robot while driving
 * \param m message containing all the distances of the obstacles detected by the robot's sensors
 * 
 * \return
 * 
 * The function is used to assist the robot while it is driving.
 * In particular the function checks that the assist drive modality in enabled (if not it quits)
 * then stores the distances value into an array with a proper dimension; then it divides this array
 * into 5 sub arrays (with dimension 144) in order to have different intervals to check.
 * Once that all the data are ready for the program, at each iteration the function checks that the
 * distance of the minimum value of the array is greater than a threshold (choosen by the programmer
 * and defined as a DEFINE at the beginning of the program); if the threshold is greater then the program
 * follows the normal flow, otherwise the function sets the linear and angular velocity to zero and
 * publish them to stop the robot thus avoiding a collision
 * Of course this step are computed for each sub array defined: the threshold may vary if choosen differently.
 * 
**/
// function to assist the robot driving
void driveAssist(const sensor_msgs::LaserScan::ConstPtr &m)
{
	// check if the manual drive is on or not
	if(assistDrive)
	{
		// declare an array of dimension of the ranges (720)
		float r[m->ranges.size()];
		
		// fill the new array to make the data available
		for(int i = 0; i < m->ranges.size(); i++)
		{
			r[i] = m->ranges[i];
		}
		
		// use all the value for the sector so divid them into 144 array (720/5)
		
		// right
		double right[SIZE];
		// front rigth
		double fr_right[SIZE];
		// front
		double front[SIZE];
		// front left
		double fr_left[SIZE];
		// left
		double left[SIZE];
		
		// fill the right
		for(int i = 0; i < SIZE; i++)
		{
			right[i] = r[i];
		}
		// fill the front right
		for(int i = 0; i < SIZE; i++)
		{
			fr_right[i] = r[i+SIZE];
		}
		// fill the front
		for(int i = 0; i < SIZE; i++)
		{
			front[i] = r[i+2*SIZE];
		}
		// fill the front left
		for(int i = 0; i < SIZE; i++)
		{
			fr_left[i] = r[i+3*SIZE];
		}
		// fill the left
		for(int i = 0; i < SIZE; i++)
		{
			left[i] = r[i+4*SIZE];
		}
		
		// check if there is an obstacle in the front
		if(min_val(front) < wth)
		{
			// if the robot has to go straight
			if(n.linear.x > 0 && n.angular.z == 0)
			{
				system("clear");
				// print a warning
				std::cout << "Wall in the front!\n";
				// stop the robot
				n.linear.x = 0;
			}
		}
		
		// check the distance on the right with the front right array
		if(min_val(fr_right) < wth)
		{
			// if the robot has to turn on the right
			if(n.linear.x > 0 && n.angular.z < 0)
			{
				system("clear");
				// print a warning
				std::cout << "Wall on the front right!\n";
				// stop the robot
				n.linear.x = 0;
				n.linear.z = 0;
			}
		}
		
		// check the distance on the right
		if(min_val(right) < wth)
		{
			// if the robot has to turn on the right
			if(n.linear.x == 0 && n.angular.z < 0)
			{
				system("clear");
				// print a warning
				std::cout << "Wall on the right!\n";
				// stop the robot
				n.linear.z = 0;
			}
		}
		
		// check the distance on the left with the front left arrat
		if(min_val(fr_left) < wth)
		{
			// if the robot has to turn on the left
			if(n.linear.x > 0 && n.angular.z > 0)
			{
				system("clear");
				// print a warning
				std::cout << "Wall on the front left!\n";
				// stop the robot
				n.linear.x = 0;
				n.linear.z = 0;
			}
		}
		
		// check the distance on the left
		if(min_val(left) < wth)
		{
			// if the robot has to turn on the left
			if(n.linear.x == 0 && n.angular.z > 0)
			{
				system("clear");
				// print a warning
				std::cout << "Wall on the left!\n";
				// stop the robot
				n.linear.z = 0;
			}
		}
		
		// publish the new velocity
		pubV.publish(n);
	}
}

/**
 * \brief Function to set the drive modality according to the user input
 * \param req, res respectively the request and response of the service
 * 
 * \return boolean true or false
 * 
 * The function is used to set the drive modality according to the user input.
 * Once the user enters an input inside the UI this is passed to the service node by
 * a service: the request is used inside the switch case to set the modality.
 * 
**/
// switch to choose what to do given a certain input
bool setDriveMod (assignment::Service::Request &req, assignment::Service::Response &res)
{
	switch(req.input)
	{	
		// publish a position (x y)
		case '1':
			system("clear");
			
			// give some instructions
			menu();

			//defining the variables to store the input
			float inX,inY;

			// get the value by the user
			std::cin >> inX >> inY;
			
			// clear the output to print again in a white background
			system("clear");
			
			// function to set the params
			setPoseParams(inX,inY);
	
			break;
			
		// drive the robot with the teleop_twist_kwyboard
		case 'm':
		case 'n':
		case 'q':
			cancelGoal();
			// call the function to drive the robot manually
			manuallyDrive(req.input);
			break;
			
		// delete the current goal
		case '3':
			// call the function to cancel the goal
			cancelGoal();
			break;
			
		// kill all nodes
		case '0':
			ros::shutdown();
			break;
			
		// invalid input
		default:
			// print
			std::cout << "Invalid input.";
			break;
	}
	
	return true;
}

/**
 * \brief Main function of the node
 * \param
 * 
 * \return
 * 
 * The main function is used to manage the node behaviour.
 * It is resppnsible of establishing the communication with the UI node and
 * making the robot move in the einvironment.
 * In particular it initializes the node handle, the publishers for the position,
 * node cancellation and the velocity when there is the user interaction.
 * The service has the aim to subscribe itself to the topics needed for the communication
 * with the other node.
 * The spi mode is done using the ros::spin() structure since there are periodic topics,
 * like the laser scan, that allow the structure.
 * 
**/
// main
int main(int argc, char ** argv)
{
	// initialising the node
	ros::init(argc, argv, "service");
	// defining a node handle
	ros::NodeHandle nh;
	
	// advertise the service
	// advertise the service and call the function
	ros::ServiceServer service = nh.advertiseService("/service", setDriveMod);
	
	// advertise topics
	// advertise the topic move_base/goal for setting the goal
	pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	
	// advertise the topic move_base/cancel for cancelling the goal
	pubCancel = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
	
	// advertise the topic cmd_vel
	pubV = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	// subscribe to the topic feedback to have the status always available and updated
	ros::Subscriber sub = nh.subscribe("/move_base/feedback", 1, takeStatus);
	
	// subscribe to the topic goal to have the current status always available and updated
	ros::Subscriber subG = nh.subscribe("/move_base/goal", 1, currGoal);
	
	// subscribe to the topic prov_cmd_vel to have the value of the velocity
	ros::Subscriber subV = nh.subscribe("/my_cmd_vel", 1, takeVel);
	
	// subscribe to the topic scan to have the value of the laser to avoid obstacles
	ros::Subscriber subL = nh.subscribe("/scan", 1, driveAssist);
	
	// spin the program
	ros::spin();
	
	return 0;
}
