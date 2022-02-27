/**
 * \file UI.cpp
 * \brief Node to manage all the UI (User Interface) actions
 * \author Matteo Maragliano
 * \version	1.0
 * \date 27/02/2022
 * 
 * 
**/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "assignment/Service.h"
#include "move_base_msgs/MoveBaseGoal.h"

// defining a client
ros::ServiceClient client;

// define a publisher 
ros::Publisher pub;

// menu to show inside the main
void menu()
{
	std::cout << "\n###################### MENU' ######################";
	std::cout << "\nPress:\n1 to publish a target with (x,y);\n2 to drive the robot with keyboard;\n3 to delete the current goal;\n0 to stop the program execution.\n";
	std::cout << "###################################################\n";
	std::cout << "\nType here: ";
}

// menu for the drive manual
void menuManual()
{
	std::cout << "###################### INFOS ######################\n";
	std::cout << "\nEnter:\nm to use assit drive\nn to not use assist drive\nq to quit\n";
	std::cout << "###################################################\n";
	std::cout << "\nType here: ";
}

// manual drive
void manuallyDrive()
{
	// defining a variable s of type final_assignment::Service
	assignment::Service s;
	// define a variable to store the user input
	char inputUsr;
	while(inputUsr != 'q')
	{
		// show the menu
		menuManual();
		// get the user choice
		std::cin >> inputUsr;
		system("clear");
		switch(inputUsr)
		{	
			// assist enabled
			case 'm':
				// print
				std::cout << "Manual drive with assistance driving enabled\n";
				// put the input on the request of the server
				s.request.input = inputUsr;
				// waut for the existance of the server
				client.waitForExistence();
				// call the server
				client.call(s);
				break;
				
			// assist disabled
			case 'n':
				// print
				std::cout << "Manual drive with assistance driving disabled\n";
				// put the input on the request of the server
				s.request.input = inputUsr;
				// waut for the existance of the server
				client.waitForExistence();
				// call the server
				client.call(s);
				break;
				
			// exit the program
			case 'q':
				// print
				std::cout << "Manual drive ended\n";
				// put the input on the request of the server
				s.request.input = inputUsr;
				// waut for the existance of the server
				client.waitForExistence();
				// call the server
				client.call(s);
				break;
				
			// invalid input
			default:
				// print
				std::cout << "Invalid input\n";
				break;
		}
	}
}

// function to show the menù and give the input
void callBack()
{
	// defining a variable s of type final_assignment::Service
	assignment::Service s;
	
	// define a variable to publish
	move_base_msgs::MoveBaseGoal goal;
	
	// defining a char to use to store the input
	char inputUsr;
	
	// show the menù
	menu();
	
	// getting the keyboard input
	std::cin >> inputUsr;
	
	// check if the input is the manual drive one
	if(inputUsr == '2')
	{
		system("clear");
		// call the function to manage the choice for the manual drive
		manuallyDrive();
	}

	// clear the output to print again in a white background
	system("clear");

	// put the input on the request of the server
	s.request.input = inputUsr;
	// waut for the existance of the server
	client.waitForExistence();
	// call the server
	client.call(s);
}

// main
int main(int argc, char ** argv)
{
	// initialising the node
	ros::init(argc, argv, "UI");
	// defining a node handle
	ros::NodeHandle nh;
	
	// call the service with the client
	client = nh.serviceClient<assignment::Service>("/service");
	
	// spin the prorgram
	// ros::spin() not used since there is not a topic to subscribe to which enables the spin mode
	
	while(ros::ok())
	{
		// call the function to manage the choice of the user for the behaviour of the robot
		callBack();
		ros::spinOnce();
	}
	
	return 0;
}
