// MTRX3760 Major Project
// CWallFollower .cpp file
// Author: Group "Our Group"

//--Include Header Files------------------
#include "CHazardHandler.h"

//--Includes------------------------------
#include <iostream>

//--CWallFollower Implementation----------

//--Constructor
CHazardHandler::CHazardHandler( CCamera* apBurgerCamera, CMotor* apBurgerMotor, CLidar* apBugerLidar,  CCartographer* apCartographer, CHazards* apHazard)
{
    // std::cout << "CTor CWallFollower" << std::endl;

    //Set number of hazards handled
    mHazardsHandled = 0;
    
    // Initialise pointers to instances of each class passed through from CBurger
    mpHazard = apHazard;
    mpBurgerCamera = apBurgerCamera;
    mpBurgerMotor = apBurgerMotor;
    mpBurgerLidar = apBugerLidar;
    mpCartographer = apCartographer;
    
    // Initalise variables for navigation
    // These variables have been derived through trial and error using the 
    // specifications from the turtlebot
    mMaintainDistance = 0.3;
    mErrorMargin = 0.1;
    mCoordinateErrorMargin = 0.2;

    mForwardVelocity = 0.15;
    mTightForwardVelocity = 0.1;
    mTurnVelocity = 0.3;
    mBroadTurnVelocity = 0.6;
    mNoVelocity = 0.0;


    // Set the position of Home initaly as 0, this will later be updated 
    // (home is not always actually 0 in hardware)
    mStationPositionX = 0;
    mStationPositionY = 0;
    mStationPositionW = 0;


    // Print inital position of home
    std::cout << "INIT STATION XYW: " << mStationPositionX << " " << mStationPositionY << " " << mStationPositionW << std::endl;

    // Set the initial state and flags
    mCurrentState = START;
    mWallToFollowFlag = 1;      // Set right wall follow
    mGetResourcesFlag = 0;      // Set resources valid 
    mRestockCompleteFlag = 1;   // Set resource restock complete 

    // Set counters to 0 
    mCounter = 0;
    mDriveForwardCounter = 0;

    // Set the Resource capacity of the robot and the other 
    mResourceCapacity = 5;      // Set capacity of Burger 
    mResourcesAvailable = mResourceCapacity;
    mHazardHandledFlag = 0;
}

//--Destructor
CHazardHandler::~CHazardHandler()
{
    //std::cout << "Dtor CWallFollower" << std::endl;
}

//--HazardHandlerLogic
void CHazardHandler::HazardHandlerLogic( int aLeftOrRight )
{   
    // Get hazard size
    mHazardSize = mpHazard->GetHazardSize();

    // Get current location values from the cartographer
    mCurrentX = mpCartographer->GetCurrentX();
    mCurrentY = mpCartographer->GetCurrentY();
    mCurrentW = mpCartographer->GetCurrentW();

    // Setting updated values for the home starting point
    if (mStationPositionX == 0 && mCurrentX != 0)
    {
        std::cout << "HEY THERE SETTING VALUES FOR STATION" << std::endl;
        mStationPositionX = mCurrentX;
        mStationPositionY = mCurrentY;
        mStationPositionW = mCurrentW;
    }

    
    // Get current lidar values based on aLeftOrRight flag
    // aLeftOrRight == 1; follow right wall
    // aLeftOrRight == -1; follow left wall
    if ( aLeftOrRight == 1 )
    {
        mDistanceFollow = mpBurgerLidar->GetCurrentRight();
        mDistanceOpposite = mpBurgerLidar->GetCurrentLeft();
    } 
    else
    {
        mDistanceFollow = mpBurgerLidar->GetCurrentLeft();
        mDistanceOpposite = mpBurgerLidar->GetCurrentRight();
    }

    //Get the distances an object is in front of it from the lidar
    mDistanceFront = mpBurgerLidar->GetCurrentFront();
    
    // Print information to user
    ROS_INFO("[CHazardHandler]: R: %.2f, F: %.2f, L: %.2f", mDistanceFollow, mDistanceFront, mDistanceOpposite);
    ROS_INFO("[CHazardHandler]: Capacity: %d / %d, Hazard Size: %d", mResourcesAvailable, mResourceCapacity , mHazardSize); 


    //State machine for the hazard handler
    switch (mCurrentState)
    {
        //Start State, starts the hazard handler and moves the state machine on the 
        // the finding hazard state
        case START:
                ROS_INFO("[CHazardHandler]: START state. --> FIND_HAZARD.");
                mCurrentState = FIND_HAZARD;
            break;

        // Turns to find the hazard, looks for an object infront of it
        case FIND_HAZARD:

            mGetResourcesFlag = 0;      // Set resources valid 
            mRestockCompleteFlag = 1;   // Set resource restock complete 
            mHazardHandledFlag = 0;     // Set hazard handled flag to 0

            // If turn country is less than required to turn 90 degress, it continues to look 
            // for the hazard. 
            if (mCounter <= 16/mBroadTurnVelocity)
            {
                ROS_INFO("[CHazardHandler]: FIND_HAZARD state. Turning to face hazard.");
                ROS_INFO("Counter updating: %d", mCounter);
                mpBurgerMotor->BurgerControl( mNoVelocity, aLeftOrRight * mBroadTurnVelocity );
                mCounter = mCounter +1 ;
            }

            //Once it has turned, moves on to the FACE_HAZARD state.
            else
            {
                ROS_INFO("[CHazardHandler]: Facing hazard --> FACE_HAZARD state.");
                mCurrentState = FACE_HAZARD;
                mCounter = 0;
            }
            break;

        // Drives the turltebot forward until it is a set distance from the hazard
        case FACE_HAZARD:

            // Tests to determine how far it is from the hazard, continues moving foward 
            // until it is within the range
            if (mDistanceFront + mErrorMargin > mMaintainDistance)
            {
                ROS_INFO("[CHazardHandler]: FACE_HAZARD state. Driving towards hazard.");
                mpBurgerMotor->BurgerControl( mForwardVelocity, mNoVelocity );
                mCounter = mCounter +1 ;
            }

            // When it is within the required range, it stops itself and moves to the next 
            // state HANDLER_HAZARD
            else
            {
                ROS_INFO("[CHazardHandler]: next to hazard --> HANDLER_HAZARD state.");
                mpBurgerMotor->BurgerControl( mNoVelocity, mNoVelocity );
                mCurrentState = HANDLER_HAZARD;
                mDriveForwardCounter = mCounter;
                mCounter = 0;
            }
            break;

        // Waits for for a specified amount of time (based on the size of the hazard 
        // that is set in CHazard
        case HANDLER_HAZARD:    

            // If the counter is less than the hazard size (*10) it waits 
            if (mCounter <= 10 * mHazardSize) 
            {                               
                ROS_INFO("[CHazardHandler]: HANDLER_HAZARD state. Handling hazrd.");
                ROS_INFO("Counter updating: %d", mCounter);
                mCounter = mCounter +1;
            }

            // When it has waited the correct amount of time, it has then handled the 
            // hazard and moves on the to next state 
            else
            {
                ROS_INFO("[CHazardHandler]: hazard handled --> GO_BACK state."); 
                mCurrentState = GO_BACK;

                //increment the amount of hazards dealt with
                mHazardsHandled = mHazardsHandled + 1;

                // Update the resources based on the amount used to deal with the hazard
                mResourcesAvailable = mResourcesAvailable - (mCounter / 10);

                mCounter = 0;
            }
            break;

        // This state reverse the turtlebot until it is basic in the position where it 
        // detected the hazard. It then moves on to the next state RESOURCE_CHECK
        case GO_BACK:

            // Drive backward the same length of time the the bot drove foward
            if (mCounter < mDriveForwardCounter)
            {
                ROS_INFO("[CHazardHandler]: GO_BACK state. Going back to path.");
                ROS_INFO("Counter updating: %d", mCounter);
                mpBurgerMotor->BurgerControl( -mForwardVelocity, mNoVelocity );
                mCounter = mCounter +1 ;
            }

            // Once that is done, move into the next state
            else
            {
                ROS_INFO("[CHazardHandler]: Back on path --> RESOURCE_CHECK state.");
                mCurrentState = RESOURCE_CHECK;
                mCounter = 0;
            }
            break;

        // This state checks the available resources the robot has, if this amount is less
        // than what is required to deal with a hazard, it will switch a indicator flag that is 
        // used by the class CWallFollower to determine the navigation method used. 
        case RESOURCE_CHECK:

            // If it doesn't have enough resoruces, its saves it position, 
            // switches the WallFollowingFlag and some other indicators and then 
            // sets the state to START
            if ( mResourcesAvailable < mHazardSize )  
            {
                mHazardPositionX = mCurrentX; 
                mHazardPositionY = mCurrentY; 
                mHazardPositionW = mCurrentW; 

                mWallToFollowFlag = -mWallToFollowFlag;
                mHazardHandledFlag = 1;
                mCounter = 0;
                mCurrentState = START;
                
                mGetResourcesFlag = 1;
                mRestockCompleteFlag = 0;
                
                ROS_INFO("[CHazardHandler]: GO_BACK state. Going home.");
            }

            // If the amount of resrouces is sufficient, the robot will continue to navigate
            // around the environment
            else
            {
                mWallToFollowFlag = mWallToFollowFlag;
                mHazardHandledFlag = 1;
                mCounter = 0;

                mCurrentState = START;

                mGetResourcesFlag = 0;
                mRestockCompleteFlag = 1;

                ROS_INFO("[CHazardHandler]: GO_BACK state. Continuing to search area.");
            }

            break;

        // This case is called when the robot has dealt with all cases and is at the end 
        case END:
            break;
    }
}

//--GetResourcesCheck
bool CHazardHandler::GetResourcesCheck()
{ 

    // Get current location values based
    mCurrentX = mpCartographer->GetCurrentX();
    mCurrentY = mpCartographer->GetCurrentY();
    mCurrentW = mpCartographer->GetCurrentW();

    // Determine if the robot is home, if it is the resources will be updated
    if (mGetResourcesFlag == 1)
    {
        // If current x,y != station x,y
        if ((mStationPositionX - mCoordinateErrorMargin <= mCurrentX) && (mCurrentX <= mStationPositionX + mCoordinateErrorMargin))
        {
            if ((mStationPositionY - mCoordinateErrorMargin <= mCurrentY) && (mCurrentY <= mStationPositionY + mCoordinateErrorMargin))
            {
                // Update resource information when it gets home (the resoruces have been refilled)
                if ((mResourcesAvailable != mResourceCapacity) && (mWallToFollowFlag < 0))
                {
                    mWallToFollowFlag = -mWallToFollowFlag;
                    mResourcesAvailable = mResourceCapacity;
                    mGetResourcesFlag = 0;
                }
            }
        }
    }
    
    //Return the resource flag 
    return mGetResourcesFlag;
}

//--ReStockCompleteCheck
bool CHazardHandler::RestockCompleteCheck()
{
    // Get current location values based
    mCurrentX = mpCartographer->GetCurrentX();
    mCurrentY = mpCartographer->GetCurrentY();
    mCurrentW = mpCartographer->GetCurrentW();
    
    //Check if the robot is home to determine if the resources have been refilled 
    if ((mGetResourcesFlag == 0) && (mHazardPositionX != 0))
    {
        if ((mHazardPositionX - 2 * mCoordinateErrorMargin <= mCurrentX) && (mCurrentX <= mHazardPositionX + 2 * mCoordinateErrorMargin))
        {
            if ((mHazardPositionY - 2 * mCoordinateErrorMargin <= mCurrentY) && (mCurrentY <= mHazardPositionY + 2 * mCoordinateErrorMargin))
            {
                // Update flag and resources
                mRestockCompleteFlag = 1;
            }
        }
    }

    // Return the flag
    return mRestockCompleteFlag;
}

//--GetWallFollowerFlag
int CHazardHandler::GetWallFollowerFlag()
{ 
    return mWallToFollowFlag;
}

//--GetHazardsHandled
int CHazardHandler::GetHazardsHandled()
{
    return mHazardsHandled;
}

//--GetHazardHandledFlag
int CHazardHandler::GetHazardHandledFlag()
{
    return mHazardHandledFlag;
}

//--GetStationX
double CHazardHandler::GetStationX()
{
    return mStationPositionX;
}

//--GetStationY
double CHazardHandler::GetStationY()
{
    return mStationPositionY;
}

//--GetStationW
double CHazardHandler::GetStationW()
{
    return mStationPositionW;
}

//--SetStationX
void CHazardHandler::SetStationX( double aCoordinate )
{
    mStationPositionX = aCoordinate;
}

//--SetStationY
void CHazardHandler::SetStationY( double aCoordinate )
{
    mStationPositionY = aCoordinate;
}

//--//--SetStationW
void CHazardHandler::SetStationW( double aCoordinate )
{
    mStationPositionW = aCoordinate;
}

