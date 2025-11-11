// MTRX3760 Major Project
// CBurger .h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CBURGER_H
#define _CBURGER_H

//---Include Header Files---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>


#include "CWallFollower.h"
#include "CHazardHandler.h"
#include "CCartographer.h"
#include "CHazards.h"

//---Includes---------------------------------------
#include <algorithm>
#include <numeric>
#include <vector>

//---CBurger Interface------------------------------
// This class CBurger is the class for the robot itself ("named
// Burger"). This class 'knows a' Wallfollower, hazardhandler, lidar, 
// motor, camera and cartographer class. 
class CBurger {

    public:
        CBurger( ros::NodeHandle& apNodeHandle, CHazards* apHazard );      //Constructor
        ~CBurger();     //Destructor

        // The function RunNavigation sets the ross communication rate (10 Hz)
        // and then runs a while loop (checking if ros of 'ok') to perform initalise
        // the robot. The function first sets the starting location as the home base
        // and then run checks on flags set within in each class to determine 
        // what it is actioning. 
        void RunNavigation();

    private:
        // ROS node handle
        ros::NodeHandle* mpNodeHandle;

        // Instances of classes known by CBurger
        CWallFollower* mpWallFollower;
        CHazardHandler* mpHazardHandler;
        
        CLidar* mpBurgerLidar;
        CMotor* mpBurgerMotor;
        CCamera* mpBurgerCamera;
        CCartographer* mpCartographer;


};


#endif // _CBURGER_H

