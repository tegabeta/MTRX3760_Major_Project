// MTRX3760 Major Project
// CLidar .cpp file
// Author: Group "Our Group"

//--Include Header Files---------------------------
#include <CLidar.h>

//--Includes---------------------------------------
#include <iostream>

//--CLidar Implementation-------------------------

//--Constructor
CLidar::CLidar(ros::NodeHandle& apNodeHandle)
{
    // std::cout << "CTor CLidar" << std::endl;
    
    // Initialise subscribe to scan 
    mScanSubscriber = apNodeHandle.subscribe("scan", 10, &CLidar::ScanCallback, this);

    // Initally assign member floats to 0
    mCurrentFront = 0.0;
    mCurrentLeft = 0.0;
    mCurrentRight = 0.0;
}

//--Destructor
CLidar::~CLidar()
{
    // std::cout << "DTor CLidar" << std::endl;
}

//--ScanCallBack
void CLidar::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& apMsg)
{
    // Extract ranges for front, left, and right
    mRangeFront.assign(apMsg->ranges.begin() + 355, apMsg->ranges.begin() + 360);
    mRangeFront.insert(mRangeFront.end(), apMsg->ranges.begin(), apMsg->ranges.begin() + 5);
    mRangeLeft.assign(apMsg->ranges.begin() + 40, apMsg->ranges.begin() + 50);
    mRangeRight.assign(apMsg->ranges.begin() + 310, apMsg->ranges.begin() + 320);
    
    // Calculate the average values for each orientation
    double sumFront = std::accumulate(mRangeFront.begin(), mRangeFront.end(), 0.0);
    double sumLeft = std::accumulate(mRangeLeft.begin(), mRangeLeft.end(), 0.0);
    double sumRight = std::accumulate(mRangeRight.begin(), mRangeRight.end(), 0.0);

    //Average the values for each direction to get a mean
    //Lidar output.
    mCurrentFront = sumFront / mRangeFront.size();
    mCurrentLeft = sumLeft / mRangeLeft.size();
    mCurrentRight = sumRight / mRangeRight.size();
}

//--GetCurrentFront
float CLidar::GetCurrentFront()
{
    return mCurrentFront;
}

//--GetCurrentLeft
float CLidar::GetCurrentLeft()
{
    return mCurrentLeft;
}

//--GetCurrentRight
float CLidar::GetCurrentRight()
{
    return mCurrentRight;
}
