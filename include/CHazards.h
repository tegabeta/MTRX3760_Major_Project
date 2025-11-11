// MTRX3760 Major Project
// CHazards.h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CHAZARDS_H
#define _CHAZARDS_H

//---Include Header Files---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include "eRobotState.h"

//---Includes---------------------------------------
#include <algorithm>
#include <numeric>

//---CHazards Interface-----------------------------
// This class stores the information about the hazard specified in main.
// This information can then be called for use.
class CHazards {

    public:
        CHazards( eHazardState aHazardType, int aHazardSize );     //Constructor
        ~CHazards();   //Destructor

        // Return the enum type of hazard 
        eHazardState GetHazardType();

        // Return the hazard size
        int GetHazardSize();

    private:

        // Values for hazard type and size
        eHazardState mHazardType;
        int mHazardSize;

};

#endif // _CHAZARDS_H