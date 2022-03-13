/**
 * \file UI.cpp
 * \brief Node to manage all the UI (User Interface) actions.
 * \author Matteo Maragliano
 * \version	1.0
 * \date 27/02/2022
 * 
 * \details
 * 
 * Subscribers to: <BR>
 * 
 * Publisher to: <BR>
 * 
 * Services: <BR>
 * /service This is the service used to pass to the service node the user choice inserted.
 * 
 * Description:
 * 
 * The node is used to manage all the UI (User Interface) actions. This is a way to have
 * them divided from the normal execution of the program. In this way there is no possibility
 * that the program waits for an input and so blocks the other processes to go on with the
 * normal running.
 * 
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "assignment/Service.h"
#include "move_base_msgs/MoveBaseGoal.h"

// defining a client
ros::ServiceClient client; ///< Definition of the client

/**
 * \brief Function to print the menù.
 * \param
 * 
 * \return
 * 
 * This function is used to print the menù from which the user can choose a way to drive the robot.
 * 
**/
// menu to show inside the main
void menu()
{
	std::cout << "\n###################### MENU' ######################";
	std::cout << "\nPress:\n1 to publish a target with (x,y);\n2 to drive the robot with keyboard;\n3 to delete the current goal;\n0 to stop the program execution.\n";
	std::cout << "###################################################\n";
	std::cout << "\nType here: ";
}

/**
 * \brief Function to print the menù for the driving assistance.
 * \param
 * 
 * \return
 * 
 * This function is used to print the menù to allow the user to choose to have the driving assistance enabled or not.
**/
// menu for the drive manual
void menuManual()
{
	std::cout << "###################### INFOS ######################\n";
	std::cout << "\nEnter:\nm to use assit drive\nn to not use assist drive\nq to quit\n";
	std::cout << "###################################################\n";
	std::cout << "\nType here: ";
}

/**
 * \brief Function to choose to have the driving assistance enabled or not.
 * \param
 * 
 * \return
 * 
 * This function allows the user to choose if the driving assistance has to be enabled or not.
 * The function processes the user coice and checks its validity; then the function enters
 * a loop till the user enters the quit-loop command.
 * Once the command has been inserted the function calls the service and pass the choice to
 * the service node in order to be performed during the program execution.
 * 
**/
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

/**
 * \brief Callback function of the main
 * \param
 * 
 * \return
 * 
 * This function is responsible of taking the user choice and sending it to the service bode.
 * Since it is inside the main it is run in an infinite loop. In particular it prints the menù,
 * after that it takes the user input; once here it checks that the choice is 2, if this condition
 * is satified then it calls the function for the driving assistance, otherwise it calls the service
 * to sends the input to the service node of the program.
 * 
**/
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

/**
 * \brief Main function of the node
 * \param
 * 
 * \return 0 if the program ends successfully
 * 
 * This is the main function of the node, responsible to make the program run.
 * First of all it initializes the node, then the node handle to manage the client and
 * eventually the subscribers if any.
 * It calls the service to have it ready for the using and then spins the callback
 * function. The spin is not performed with the ros::spin() structure since there is 
 * not topic to subscribe to that allows this action.
 * 
**/
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
