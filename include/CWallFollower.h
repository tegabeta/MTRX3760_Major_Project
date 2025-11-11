// MTRX3760 Major Project
// CWallFollower .h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CWALLFOLLOWER_H
#define _CWALLFOLLOWER_H

//---Include Header Files---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include "CLidar.h"
#include "CCamera.h"
#include "CMotor.h"
#include "CCartographer.h"

//---Includes---------------------------------------
#include <vector>

//---CWallFollower Interface------------------------
// The class CWallFollower is the navigation logic for the turtlebot.
// The class takes in the information from the lidar and camera and then makes decisions
// based on that input using a state machine.
// The class also both recieves informatation from the odom and then 
// pass instructions through the the class CMotor based on the State 
// Machine logic. 
// The navigation can navigate either left or right wall following depending
// on the mode the robot is in.
class CWallFollower {

public:
    CWallFollower( CCamera* apBurgerCamera, CMotor* apBurgerMotor, CLidar* apBurgerLidar, CCartographer* apCartographer );     //Constructor
    ~CWallFollower();   //Destructor

    // The WallFollowLogic function serves as the central logic hub
    // for the robot's behavior. It operates as a finite state machine
    // with states: INIT, FOLLOW_WALL, ROTATE, AVOID_OBSTACLE, PAUSE and STOP.
    // Depending on the current state, the function issues movement commands to the robot.
    // These commands are then published to the cmd_vel topic for the robot to execute.
    void WallFollowLogic( int aLeftOrRight );

    // Used to set the state once a object is detected
    void SetCurrentState( eWallFollowerState aRobotState );

    // Returns the value for the home base held within the class
    double GetStationX();
    double GetStationY();
    double GetStationW();

    //Resets the flags used by the module
    void SetRestockCompleteCheck( int aFlag );
    void SetHazardsHandled( int aSum );

private:

    // Initalises distances that will be used in the nagivation logic.
    // These values will be initalised in the constructor as the values
    // they should be for this logic
    float mMaintainDistance;
    float mErrorMargin;
    float mCoordinateErrorMargin;

    // Values to be used to pass information to the Motor class
    float mForwardVelocity;
    float mTightForwardVelocity;
    float mTurnVelocity;
    float mBroadTurnVelocity;
    float mNoVelocity;

    // Values for the counters used to turn 
    int mCounter;
    int mPauseCounter;

    // Flags used to handle hazards and check resource levels
    bool mRestockCompleteFlag;
    int mHazardsHandled;

    // ENUM information about the states used in the 
    // by the state machine
    eWallFollowerState mCurrentState;
    eWallFollowerState mPreviousState;
    
    // Initalise double for values from cartographer
    double mCurrentX;
    double mCurrentY;
    double mCurrentW;

    // Values to store the location of the home base
    double mStationPositionX; 
    double mStationPositionY; 
    double mStationPositionW; 

    // Initalise float for values from lidar
    float mDistanceFront;
    float mDistanceFollow;
    float mDistanceOpposite;

    // Pointers for instances of a classes that are passed to 
    // the class in the constructor
    CCamera* mpBurgerCamera;
    CMotor* mpBurgerMotor;
    CLidar* mpBurgerLidar;
    CCartographer* mpCartographer;

};

#endif // _CWALLFOLLOWER_H
