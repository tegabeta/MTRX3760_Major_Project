// MTRX3760 Major Project
// CCartographer .h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CCARTOGRAPHER_H
#define _CCARTOGRAPHER_H

//---Include Header Files---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

//---Includes---------------------------------------
#include <algorithm>
#include <numeric>

//---CCartographer Interface------------------------
// The CCartographer uses the AmclCallback function to obtain 
// the current position (x,y) and orientation (w) of the burger.
class CCartographer {

    public:
        
        CCartographer(ros::NodeHandle& apNodeHandle);   //Contructor
        ~CCartographer();                               //Destructor
        
        
        // The AmclCallback function is triggered when a new Geometry message is received.
        // This function processes the Geometry data to determine the position (x, y, w) and 
        // orientation of the burger in 2D space. The Geometry data is saved as a pointer apMsg.      
        void AmclCallback(const nav_msgs::Odometry::ConstPtr &apMsg);  
        
       // Returns the position and orinetation values
        double GetCurrentX();
        double GetCurrentY();
        double GetCurrentW();

    private:
        
        // Initalise doubles to store the burger's current location.
        double mPositionX; 
        double mPositionY; 
        double mPositionW; 
        
        //Initalise a subscriber
        ros::Subscriber mAmclSub;


};

#endif // _CCARTOGRAPHER_H