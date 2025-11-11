// MTRX3760 Major Project
// main.cpp file
// Author: Group "Our Group"

//--Include Header Files---------------------------
#include "CBurger.h"
#include "CHazards.h"

//--Includes---------------------------------------
#include <ros/ros.h>

//---Main------------------------------------------
// Main initalise ROS and creates node handle. The type of hazard 
// been dealt with by the robot is set as well as the 'size' of the 
// hazard. Class CBurger is then initialised and the function RunNavigation
// from the class if called in order to run the system. 
int main(int argc, char** argv) {

    // Initialize ROS and create a node handle
    ros::init(argc, argv, "follow_the_wall");
    ros::NodeHandle rosNodeHandler;

    // Create a hazards object
    CHazards hazard( FIRE, 2 );    // Takes an eHazadState and size of hazard

    // Create CBurger object
    CBurger burgerMobile( rosNodeHandler, &hazard );

    // Run Navigation
    burgerMobile.RunNavigation();

    return 0;
}
