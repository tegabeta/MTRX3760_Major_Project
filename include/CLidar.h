// MTRX3760 Major Project
// CLidar .h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CLIDAR_H
#define _CLIDAR_H

//---Include Header Files---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>

//---Includes---------------------------------------
#include <algorithm>
#include <numeric>
#include <vector>

//---CLidar Interface-------------------------------
// The class CLidar is able to take the information
// from the Lidar, averaged it out and then output it when
// a function is called for that specific piece of information.
class CLidar {

    public:
        
        CLidar(ros::NodeHandle& apNodeHandle);  //Contructor
        ~CLidar();                              //Destructor
        
        
        // The ScanCallback function is triggered when a new LaserScan message is received.
        // This function processes the LaserScan data to determine the distance to objects
        // in front, to the right, and to the left of the robot.
        // The LaserScan data is saved as a pointer pMsg.
        //
        // Specifically, it:
        // 1. Extracts a set of range values from the LaserScan message for each orientation
        // (front, right, left).
        // 2. Calculates the average distance to obstacles for each orientation.
        // 3. Stores these average values in class member variables for use in navigation logic.
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& apMsg);
        
        // Returns the value
        float GetCurrentFront();
        float GetCurrentLeft();
        float GetCurrentRight();

    private:
        
        // Initalise a vector of floats to be used by the fucntion
        // ScanCallback to take in a range of values to be averaged out
        std::vector<float> mRangeFront;
        std::vector<float> mRangeLeft;
        std::vector<float> mRangeRight;

        // Initalise float values that will be used to pass
        // back the Lidar information to the navigation logic
        float mCurrentFront;
        float mCurrentLeft;
        float mCurrentRight;
        
        //Initalise a subscriber
        ros::Subscriber mScanSubscriber;
};

#endif // _CLIDAR_H


