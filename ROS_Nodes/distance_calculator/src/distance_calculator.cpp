/*!
 * \file distance_calculator.cpp
 * \brief Tracks the distance travelled by the robot and publishes it to the distance_travelled topic
 *
 * Tracks the distance a robot has travelled, by examining the odometry data and comparing it to previous
 * odometry data
 * @param 1 = Name of the Robot that is being used
 *
 * \author Chris Rhodan rhodanchris@gmail.com	
 * \date 29 Sep, 2014
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <distance_calculator/distance_travelled_msg.h>
#include "distance_calculator/ResetCount.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <time.h>
#include <signal.h>
#include <string>

// Variable to track the total distance travelled so far, this will be converted into the ROS message
// distance_travelled for publication whenever it is updated
double distanceTravelled = 0;

// Create a new ROS Publisher and 
ros::Publisher pub;

// Create the message that will hold the distance travelled and the name of the robot publishing that data, 
// this message is what will actually be published
distance_calculator::distance_travelled_msg distance_travelled;

// Create a Vector that will hold the coordinates of the last position of the robot that will be used to compare
// to the current position of the robot
std::vector<float> previousPosition(3);

// Tracks whether this is the first time the callback is called, if it is the first time then it won't do the 
// Calculation to find the difference in distance 
int firstTime = 1;


int test = 0;


// Calculates the difference (in m) between the current position of the robot and previous position of the robot
double CalculateDistance(const nav_msgs::Odometry::ConstPtr& msg){
	// Value to store the combined distances (that will be square rooted)
	double totalDifference = 0;

	// Add the X Difference Squared
	totalDifference += pow(msg->pose.pose.position.x - previousPosition.at(0), 2.0);

	// Add the Y Difference Squared
	totalDifference += pow(msg->pose.pose.position.y - previousPosition.at(1), 2.0);

	// Add the Z Difference Squared
	totalDifference += pow(msg->pose.pose.position.z - previousPosition.at(2), 2.0);


	// Return the difference between each vector by taking the square root of the total difference
	return sqrt(totalDifference);
}

// Logs the current time and current distance travalled to a csv file
void RecordDistanceToFile(){
	// Create a new OFStream for our CSV file
	std::ofstream out;

	// Get the current time
	time_t now = time(0);

	// Create a time struct and set it to the current time
	struct tm timeStruct = *localtime(&now);

	// Buffer to hold time as a string
	char buf[128];

	// Convert the time struct into a string using strftime
	// Format for time is hours:minutes:seconds Year:Month:Day
	strftime(buf, sizeof(buf), "%X %Y-%m-%d", &timeStruct);

	// Open the CSV File in Append Mode
	out.open("distance_calculator.csv", std::ios::app | std::ios_base::out);

	// Write the date and distance travelled to the end of the file

	out << buf << "," << distanceTravelled << "\n";

	// Close the CSV File
	out.close();

}

// Service call to reset the current count
bool ResetDistanceCount(distance_calculator::ResetCount::Request &req,
						distance_calculator::ResetCount::Response &res){
	
	// Log the Distance travelled to a file
	RecordDistanceToFile();

	// Reset the count
	distanceTravelled = 0;

	return true;
}


// If the ROS node is closed through SIGINT (ctrl-c) then this function will catch the interrupt
// and call the RecordDistanceToFile function before closing the node
void SIGINT_Handler(int s){

	// Log Distance to the file
	RecordDistanceToFile();

	// Shutdown ROS node
	ros::shutdown();
}

void DistanceCalculatorCallback(const nav_msgs::Odometry::ConstPtr& msg){
	
	// If its the first time it's been called, then don't calculate the distance (as there is no previous
	// location yet)
	if(!firstTime){
		// Calculate the distance between the previous location and current location
		double value = CalculateDistance(msg);

		// Add the Distance from the previous location to the total distance travelled
		distanceTravelled += value;
	}
	else{
		// Next time won't be the first time this function will be called, so set firstTime to false
		firstTime = 0;
	}

	// Convert the total distance travelled into the ROS message and then publish it
	distance_travelled.distance_travelled.data = distanceTravelled;
	pub.publish(distance_travelled);

	// Save the current position as the previous position
	previousPosition.at(0) = msg->pose.pose.position.x;
	previousPosition.at(1) = msg->pose.pose.position.y;
	previousPosition.at(2) = msg->pose.pose.position.z;
}

// Loads previously stored data from a saved csv file (if it exists)
void LoadPreviousFileData(){

	// Create a new IFStream for our CSV file
	std::ifstream in;

	// Open the CSV file
	in.open("distance_calculator.csv");

	// If the file was opened correctly, read from the file. If not,
	// then ignore the function and just close the file
	if(in.good()){
		// token to hold each segment of the line
		char* token;

		// string to hold the lastLine of data
		std::string line;

		// Read all lines in the file until we reach the last line of the csv file 
		// (as this will be the most recent entry)
		while (in >> std::ws && std::getline(in, line)){
			// Do nothing, just keep looping
		};

		// At this point, lastLine should hold the last line of data but we need to convert it to a
		// character pointer in order to use strtok
		char* lastLine = strdup(line.c_str());

		// Get the distance stored in the csv file. Note we call this twice as the first time
		// wll give us the data/time it was updated which isn't used in this program. 
		token = strtok(lastLine, ",");
		token = strtok(NULL, ",");

		// Convert to double and update the distance travelled value
		distanceTravelled = atof(token);
	}
	
	// Close the file
	in.close();
}


int main(int argc, char ** argv)
{
	// Load the previous distance_calculator data
	LoadPreviousFileData();

	// Initialize the ros node for use
	ros::init(argc, argv, "distance_calculator");

	// Node Handle for the distance_calculator Node that will be used
	ros::NodeHandle nh;

	// Advertise the distance travelled as a distance_travelled_msg
	pub = nh.advertise<distance_calculator::distance_travelled_msg>("distance_calculator/distance_travelled", 2, true);

	// Create and advertise the service
	ros::ServiceServer service = nh.advertiseService("distance_calculator/ResetDistanceCount", ResetDistanceCount);

	// Add the name of the robot to the published topic based on the first argument passed
	distance_travelled.robot_name = argv[1];

	// Convert the initial distanceTravelled to a ROS Message and publish the initial distanceTravelled value of 
	// 0 so we have a starting point for subscribers to access
	distance_travelled.distance_travelled.data = distanceTravelled;
	pub.publish(distance_travelled);

	// Create a new ROS Subscriber that will suscribe to the odometry topic being broadcast from the robot
	ros::Subscriber sub = nh.subscribe("odom", 1000, DistanceCalculatorCallback);

	// Create a sigaction struct to handle SIGINT interrupt
	struct sigaction sigIntHandler;

	// Set the handler to our SIGINT_handler function
	sigIntHandler.sa_handler = SIGINT_Handler;

	// Set the Interrupt as off
	sigemptyset(&sigIntHandler.sa_mask);

	// Disbale any additional flags
	sigIntHandler.sa_flags = 0;

	// Redirect any SIGINT calls to our sigIntHandler
	sigaction(SIGINT, &sigIntHandler, NULL);

	// Ros_info output just to let the user know it has started up without error
	ROS_INFO("distance_calculator Initialised");

	ros::spin();
	return 0;
}