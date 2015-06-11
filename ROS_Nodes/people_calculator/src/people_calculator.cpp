/*!
 * \file people_calculator.cpp
 * \brief Calculates how many people the Robot has seen and publishes this information to a topic
 *
 * Tracks the number of people seen by the robot over time, excluding duplicate people or those
 * who are of a low reliability, and publishes this information to the people_seen topic
 * @param 1 = Name of the Robot that is being used
 *
 * \author Chris Rhodan rhodanchris@gmail.com	
 * \date 16 Sep, 2014
 */

#include <ros/ros.h>
#include <people_msgs/People.h>
#include <people_calculator/people_seen_msg.h>
#include <iostream>
#include <vector>
#include "people_calculator/ResetCount.h"
#include <fstream>
#include <time.h>
#include <signal.h>
#include <string>

/*
 *
 *	Global Variables
 *
 */

// Initialize the variable to hold number of people seen
double peopleSeen = 0;

// Create a Vector which will hold the names of people seen in the previous message. This will be used to ensure we're 
// not trakcing the same person repeatedly; they have to leave the robot's vision first 
// Note: "Name" is actually a unique int id, rather than an actual name (which would be a string or char)

std::vector<int> peopleSeenPreviously;

// Declare the publisher which will broadcast the people_seen_msg
ros::Publisher pub;

// Create the message that will hold the people seen and the name of the robot publishing that data, 
// this message is what will actually be published
people_calculator::people_seen_msg people_seen;


// Logs the current time, number of people seen and the list of people seen previously to a csv file
void RecordPeopleToFile(){
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
	out.open("people_calculator.csv", std::ios::app | std::ios_base::out);

	// Write the date and number of people seen, seperated by a comman which is standard for csv files
	out << buf << "," << peopleSeen;

	// For each entry in the peopleSeenPreviously vector, add them to the csv line, again seperated by a 
	// comma which is the convention for csv files
	for(int i = 0; i < peopleSeenPreviously.size(); i++){
		out << "," << peopleSeenPreviously.at(i);
	}

	// End the line with a new line character
	out << "\n";

	// Close the CSV File
	out.close();

}


// If the ROS node is closed through SIGINT (ctrl-c) then this function will catch the interrupt
// and call the RecordPeopleToFile function before closing the node
void SIGINT_Handler(int s){

	// Log Distance to the file
	RecordPeopleToFile();
	
	// Shutdown ROS node
	ros::shutdown();
}


// Service call to reset the current count
bool ResetPeopleCount(people_calculator::ResetCount::Request &req,
					  people_calculator::ResetCount::Response &res){
	
	// Record the people seen to file
	RecordPeopleToFile();

	// Reset the count to zero
	peopleSeen = 0;

	return true;
}


void peopleSubscriberCallback(const people_msgs::People::ConstPtr& msg){

	// As we'll be using the number of people more than once, it is more efficient to store the value in a variable
	int numberPeople = msg->people.size();

	// Create an array that will store the names of each person seen in this message	
	std::vector<int> peopleInMessage(numberPeople);


	for (int i = 0; i < numberPeople; i++){

		// Initialise the person's name to -1, which will indicate they can't be identified reliably enough
		peopleInMessage.at(i) = -1;

		// Only include those that have a better than 50% reliability
		if(msg->people[i].reliability > 0.5){

			// Store the person's name as it will be used more than once
			int personName = atoi(msg->people[i].name.c_str());

			// Store the person's name as someone in the message
			peopleInMessage.at(i) = personName;

			// Check the person wasn't seen in the previous message (i.e. check we're not adding someone who has
			// already been counted)
			if(std::find(peopleSeenPreviously.begin(), peopleSeenPreviously.end(), personName) == peopleSeenPreviously.end()){
				
				// Increment the number of people seen
				peopleSeen++;

			}
		}
	}
	
	// Update the list of the names of people seen previously
	peopleSeenPreviously = peopleInMessage;


	// Publish an updated message
	people_seen.people_seen.data = peopleSeen;
	pub.publish(people_seen);
	
}


// Loads previously stored data from a saved csv file (if it exists)
void LoadPreviousFileData(){

	// Create a new IFStream for our CSV file
	std::ifstream in;

	// Open the CSV file
	in.open("people_calculator.csv");

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

		// Get the number of people stored in the csv file. Note we call this twice as the first time
		// wll give us the data/time it was updated which isn't used in this program. 
		token = strtok(lastLine, ",");
		token = strtok(NULL, ",");

		// Convert to double and update the people seen value
		peopleSeen = atof(token);
		// int to track the index of the peopleSeenPreviously vector
		int index = 0;

		// We don't know how many people were seen previously so loop through until we reach the end of
		// the line (strtok returns NULL when this occurs).

		// Get the next token
		token = strtok(NULL, ",");

		while(token != NULL){	
			// Convert token to an int and add it to the peopleSeenPreviously vector
			//peopleSeenPreviously.at(index) = 346;//atoi(token);
			peopleSeenPreviously.push_back(atoi(token));

			// Increment index
			index++;

			// Get the next token
			token = strtok(NULL, ",");
		}
	}
	
	// Close the file
	in.close();
}


int main(int argc, char ** argv)
{
	// Load the previous people_calculator data
	LoadPreviousFileData();
	
	// Initialize the ros node for use
	ros::init(argc, argv, "people_calculator");

	// Node Handle for the people_calculator Node that will be used
	ros::NodeHandle nh;

	// Initialize the ROS Publisher and Advertise the people seen message as a people_seen_msg
	pub = nh.advertise<people_calculator::people_seen_msg>("people_calculator/people_seen", 2, true);

	// Create a new ROS Subscriber that will suscribe to the People topic being broadcast from the robot
	ros::Subscriber sub = nh.subscribe("people", 1000, peopleSubscriberCallback);

	// Create and advertise the service
	ros::ServiceServer service = nh.advertiseService("people_calculator/ResetPeopleCount", ResetPeopleCount);
	
	// Add the name of the robot to the published topic based on the first argument passed
	people_seen.robot_name = argv[1];

	// Convert the initial peopleSeen to a ROS Message and publish the initial peopleSeen value of 
	// 0 so we have a starting point for subscribers to access
	people_seen.people_seen.data = peopleSeen;
	pub.publish(people_seen);

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

	// Initialise Message to let the user know everything loaded correctly
	ROS_INFO("people_calculator Initialised");

	ros::spin();
	return 0;
}