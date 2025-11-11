// MTRX3760 Major Project
// CHazardHandler .h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CHAZARDHANDLER_H
#define _CHAZARDHANDLER_H

//---Include Header Files---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include "CLidar.h"
#include "CCamera.h"
#include "CMotor.h"
#include "CCartographer.h"
#include "CHazards.h"

//---Includes---------------------------------------
#include <algorithm>
#include <numeric>

//---CHazardHandler Interface----------------------- 
// The class CHarzardHandler is the control logic for the turtlebot handling 
// the hazard specified from the user selection in the main function. 
// This class knows the camera, motor, lidar and cartographer classes as well as
// the specifications of each hazard type. 
// The class gets told that that there is a hazard from the camera module and 
// is able to move to the hazard, action the method of mitigating the hazard (this could
// be an added functionality due to the modulisation of the code) and also checks how 
// many resources have been used / if it needs to go home to replenish.
// If it needs to go home, a flag is switched that tells the wallfollower logic 
// to go home.
class CHazardHandler {

    public:
        CHazardHandler( CCamera* apBurgerCamera, CMotor* apBurgerMotor, CLidar* apBurgerLidar, CCartographer* apCartographer, CHazards* apHazard );     //Constructor
        ~CHazardHandler();   //Destructor

        // This function calls GetHazardSize from the class CHarzards to determine
        // what resources are required to deal with the hazard. It then runs a state
        // machine to handle the hazard (START, FIND_HAZARD, FACE_HAZARD, HANDLE_HAZARD
        // GO_BACK, RESOURCE_CHECK). This function also is able to change a flag to 
        // indicate to the class CWallFollower if it should continue right wall following
        // (continuing through the situation) or start left wall following (going home)
        // NOTE: See the .cpp file throughout the state machine for a more detailed 
        // explaination
        void HazardHandlerLogic( int aLeftOrRight );

        // This function gets the current position of the robot and determines if it 
        // is at 'home'. If it is, it then 'refills' the resources and changes the navigation
        // flag ensuring the turtlebot is ready to once again deal with hazards
        // A bool flag is returned to indicate that state the process is at (e.g. has it made it 
        // home and refilled its resources)
        bool GetResourcesCheck();

        // This function gets the current position of the robot to check if the 
        // restocking of its resources has been completed. If it has the bool value
        // is switch and returned.
        bool RestockCompleteCheck();

        // These functions return this information in the form of an int
        int GetWallFollowerFlag();
        int GetHazardsHandled();
        int GetHazardHandledFlag();

        // These functions return this information in the form of an double
        double GetStationX();
        double GetStationY();
        double GetStationW();

        // These functions set the position of the station or home base
        void SetStationX( double aCoordinate );
        void SetStationY( double aCoordinate );
        void SetStationW( double aCoordinate );


    private:
        // Variables utilised for hazard handling 
        int mResourceCapacity;  // Amount of resources bot can take
        int mResourcesAvailable;    //Amount of resources currently available
        int mHazardSize;    // Size of hazard 
    
        int mWallToFollowFlag;  // Value inidicating to CWallFollower what navigation mode 
                                // should be in.
        int mHazardsHandled;    // Number of hazards handled
        int mHazardHandledFlag; //Flag indicating that a hazard has been handled

        // Flags indicating if the resources refill has been completed / 
        // if it is occuring
        bool mGetResourcesFlag; 
        bool mRestockCompleteFlag;


        // Initalises distances that will be used in the nagivation logic.
        // These values will be initalised in the constructor as the values
        // they should be for this logic
        float mMaintainDistance;
        float mErrorMargin;
        float mCoordinateErrorMargin;

        // Values for the veleocity that will be passed to the class CMotor
        float mForwardVelocity;
        float mTightForwardVelocity;
        float mTurnVelocity;
        float mBroadTurnVelocity;
        float mNoVelocity;

        // Vaues used to track amount of time robot spends doing an 
        // acitivity
        int mCounter;
        int mDriveForwardCounter;
        int mRestockCounter;

        // Initalise float for values from lidar
        float mDistanceFront;
        float mDistanceFollow;
        float mDistanceOpposite;

        // Initalise double for values from cartographer
        double mCurrentX;
        double mCurrentY;
        double mCurrentW;

        // Initalise double values to store relevant location.
        double mHazardPositionX; 
        double mHazardPositionY; 
        double mHazardPositionW;

        // Values for the position of home base.
        double mStationPositionX; 
        double mStationPositionY; 
        double mStationPositionW; 
        
        // ENUM information about the states used in the 
        // by the state machine
        eHazardHandlerState mCurrentState;
        
        // Pointers for instances of a classes that are passed to 
        // the class in the constructor
        CCamera* mpBurgerCamera;
        CMotor* mpBurgerMotor;
        CLidar* mpBurgerLidar;
        CCartographer* mpCartographer;
        CHazards* mpHazard;

};

#endif // _CHAZARDHANDLER_H