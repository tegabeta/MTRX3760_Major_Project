// MTRX3760 Major Project
// CCartographer .cpp file
// Author: Group "Our Group"

//--Include Header Files---------------------------
#include <CCartographer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

//--Includes---------------------------------------
#include <iostream>

//--CCartographer Implementation-------------------------

//--Constructor
CCartographer::CCartographer(ros::NodeHandle& apNodeHandle)
{
    // std::cout << "CTor CCartographer" << std::endl;
    
    // Initialise subscribe to scan the odom
    mAmclSub = apNodeHandle.subscribe("odom", 10, &CCartographer::AmclCallback, this);

    // Initally assign member floats to 0
    mPositionX = 0.0;
    mPositionY = 0.0;
    mPositionW = 0.0;
}

//--Destructor
CCartographer::~CCartographer()
{
    // std::cout << "DTor CCartographer" << std::endl;
}

//--AmclCallback
void CCartographer::AmclCallback(const nav_msgs::Odometry::ConstPtr &apMsg) 
{
    // Extract coordinates for x, y, and w
    mPositionX = apMsg->pose.pose.position.x;
    mPositionY = apMsg->pose.pose.position.y;
    mPositionW = apMsg->pose.pose.orientation.w;   

}

//--GetCurrentX
double CCartographer::GetCurrentX()
{
    return mPositionX;
}

//--GetCurrentY
double CCartographer::GetCurrentY()
{
    return mPositionY; 
}

//--GetCurrentW
double CCartographer::GetCurrentW()
{
    return mPositionW; 
}

