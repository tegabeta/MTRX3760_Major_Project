// MTRX3760 Major Project
// CWallFollower .cpp file
// Author: Group "Our Group"

//--Include Header Files------------------
#include "CWallFollower.h"

//--Includes------------------------------
#include <iostream>

//--CWallFollower Implementation----------

//--Constructor
CWallFollower::CWallFollower( CCamera* apBurgerCamera, CMotor* apBurgerMotor, CLidar* apBugerLidar,  CCartographer* apCartographer )
{
    // std::cout << "CTor CWallFollower" << std::endl;
    
    // Initialise pointers to instances of each class passed through from CBurger
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


    // Set counter to 0 
    mCounter = 0;

    // Set inital position as 0 
    mStationPositionX = 0;
    mStationPositionY = 0;
    mStationPositionW = 0;

    // Set the initial state
    mCurrentState = INIT;

}

//--Destructor
CWallFollower::~CWallFollower()
{
    //std::cout << "Dtor CWallFollower" << std::endl;
}

// //--WallFollowLogic
void CWallFollower::WallFollowLogic( int aLeftOrRight )
{
    // Variable used to change between states for adjusting the burger 
    int stateFlip = 0;
    
    // Get the location of the home based at the beginning of the code.
    mCurrentX = mpCartographer->GetCurrentX();
    mCurrentY = mpCartographer->GetCurrentY();
    mCurrentW = mpCartographer->GetCurrentW();

    // If the inital values set in the constructor are still zero, update them.
    // (This was required because the values of the inital location of the bot 
    // was not zero)
    if (mStationPositionX == 0)
    {
        mStationPositionX = mCurrentX;
        mStationPositionY = mCurrentY;
        mStationPositionW = mCurrentW;
    }

    // Get current lidar values based on aLeftOrRight flag
    // 1; follow right wall | -1; follow left wall
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

    // Get the distance of the next object infront of the lidar
    mDistanceFront = mpBurgerLidar->GetCurrentFront();


    // Print information for the user
    ROS_INFO("[CWallFollower]: Near: %.2f, Front: %.2f, Opp.: %.2f", mDistanceFollow, mDistanceFront, mDistanceOpposite);
    ROS_INFO("[CWallFollower]: X Range Check: %.2f <= %.2f <= %.2f", mStationPositionX - mCoordinateErrorMargin, mCurrentX, mStationPositionX + mCoordinateErrorMargin );
    ROS_INFO("[CWallFollower]: Y Range Check: %.2f <= %.2f <= %.2f", mStationPositionY - mCoordinateErrorMargin, mCurrentY, mStationPositionY + mCoordinateErrorMargin );
    
    // If the robot has returned to home by going around the whole environment as opposed 
    // to returning home for resources, move it to the stop state - the robot has completed its
    // task
    if ((mStationPositionX - mCoordinateErrorMargin <= mCurrentX) && (mCurrentX <= mStationPositionX + mCoordinateErrorMargin))
    {
        if ((mStationPositionY - mCoordinateErrorMargin <= mCurrentY) && (mCurrentY <= mStationPositionY + mCoordinateErrorMargin))
        {
            if ((mHazardsHandled > 0) && (mRestockCompleteFlag != 0))
            {
                mCurrentState = STOP;
            }
            
            // If the robot is just pausing, set current state and continue
            else if ((mHazardsHandled > 0) && (mPauseCounter == 0))
            {
                mCurrentState = PAUSE;
                mPauseCounter = 1;
            }
        }
    }

    // Set pause counter as zero if not at home 
    else
    {
        mPauseCounter = 0;
    }



    // This is a state machine to control how the robot navigates through the environemnt. 
    // The comments below indicate how the state machine works.
    switch (mCurrentState)
    {
        // Initalise state, the robot will either follow the fall or turn to find the closest 
        // one depending on the information from the lidar. 
        case INIT:
            // Check if close enough to wall, if it is switch to the FOLLOW_WALL state
            if ((mDistanceFollow < mMaintainDistance) && (mDistanceFollow != 0))
            {
                ROS_INFO("[CWallFollower]: --> FOLLOW_WALL state.");
                mCurrentState = FOLLOW_WALL;
            }

            // If it is not close enough to a wall, turn to find a wall. 
            else
            {
                ROS_INFO("[CWallFollower]: INIT. Turning to find wall.");
                mpBurgerMotor->BurgerControl( mNoVelocity, aLeftOrRight * -mBroadTurnVelocity );
            }
            break;


        // Follow wall state keeps the robot following the wall
        case FOLLOW_WALL:
            ROS_INFO("[CWallFollower]: FOLLOW_WALL");

            // If nothing infront of burger, continue to follow the wall
            if ((mDistanceFront > mMaintainDistance) || (mDistanceFront <= 0.01))
            {
                // Too close to wall; adjust opposite way
                if (mDistanceFollow < mMaintainDistance - mErrorMargin)
                {
                    ROS_INFO("Range: %.2f m - Too close. Turn away.", mDistanceFollow);
                    mpBurgerMotor->BurgerControl( mForwardVelocity/2, aLeftOrRight * mTurnVelocity );
                }

                // Close enough to wall to track; continue straight
                else if (mDistanceFollow >= (mMaintainDistance - mErrorMargin) && mDistanceFollow < mMaintainDistance)
                {
                    ROS_INFO("Range: %.2f m - Good. Continue straight.", mDistanceFollow);
                    mpBurgerMotor->BurgerControl( mForwardVelocity, mNoVelocity );
                }
                
                // Too far from wall; adjust close 1
                else if (stateFlip != 0)
                {
                    ROS_INFO("Range: %.2f m - Too far; turn towards.", mDistanceFollow);
                    mpBurgerMotor->BurgerControl( mNoVelocity, aLeftOrRight * -mTurnVelocity );
                    stateFlip = 1;
                }

                // Too far from wall; adjust close 2
               else
                {
                    ROS_INFO("Range: %.2f m - Too far; turn towards.", mDistanceFollow);
                    mpBurgerMotor->BurgerControl( mTightForwardVelocity, aLeftOrRight * -mTurnVelocity );
                    stateFlip = 0;
                }
            }

            // If there is something infront of the robot, the state will be changed depending on
            // if the robot is returning home or actively searching for hazards
            else
            {             
                // If the robot is search for hazards (not going home), it will rotate to 
                // look at the obstacle to determine if there is a hazard on it, 
                if ( (aLeftOrRight == 1) && (mRestockCompleteFlag != 0))       
                {
                    ROS_INFO("Obstacle detected --> ROTATE state.");
                    mCurrentState = ROTATE;
                    mPreviousState = FOLLOW_WALL;
                } 

                // If the robot is returning home, or coming back to the position it 
                // ran out of resources, it will not check the wall infront for a
                // hazard
                else
                {
                    ROS_INFO("Obstacle detected --> AVOID_OBSTACLE state.");
                    mCurrentState = AVOID_OBSTACLE;
                    mPreviousState = FOLLOW_WALL;
                }
            }
            break;

        // This state rotates the robot to look with the camera at an obstacle to determine 
        // if there is a hazard on it. 
        case ROTATE:

            // Rotate 90 degrees 
            if (mCounter <= (16/mBroadTurnVelocity))
            {
                ROS_INFO("[CWallFollower]: ROTATE state. Rotating camera to face obsticale");
                ROS_INFO("Counter updating: %d", mCounter);
                mpBurgerMotor->BurgerControl( mNoVelocity, aLeftOrRight * -mBroadTurnVelocity );
                mCounter = mCounter +1 ;
            }
            
            // Once the rotate has been done, the obstacle can now be avoided. Go to the 
            // AVOID_OBSTACLE state 
            else
            {
                ROS_INFO(" Hazard check done --> AVOID_OBSTACLE state.");  
                mCurrentState = AVOID_OBSTACLE;
                mPreviousState = ROTATE;
                mCounter = 0;
            }
            break;
        
        // This state allows the robot to avoid an obstacle that is in its path 
        case AVOID_OBSTACLE:

            // If the robot has rotated to look at the obstacle in the previous state,
            // it will then turn 180 degrees to allow it to then continue on its way (90
            // to avoid the obstacle and 90 to account for the fact it turned the wrong way
            // to show the camera to the obstacle)
            if ( mPreviousState == ROTATE ) 
            {
                // Turn for the right count
                if (mCounter <= 31/mBroadTurnVelocity)
                {
                    ROS_INFO("[CWallFollower]: AVOID_OBSTACLE state. Turning back to path.");
                    ROS_INFO("Counter updating: %d", mCounter);
                    mpBurgerMotor->BurgerControl( mNoVelocity, aLeftOrRight * mBroadTurnVelocity );
                    mCounter = mCounter +1 ;
                }

                // Once count complete, continue following to wall using the 
                // state FOLLOW_WALL
                else
                {
                    ROS_INFO("[CWallFollower]: Obstacle cleared --> FOLLOW_WALL state.");
                    mCurrentState = FOLLOW_WALL;
                    mCounter = 0;
                }
            } 

            // If the robot did not show its camera to the obstacle, only turn 90 degrees
            else
            {
                // continue to turn for the right count
                if (mCounter <= 16/mBroadTurnVelocity)
                {
                    ROS_INFO("[CWallFollower]: AVOID_OBSTACLE state. Turning back to path.");
                    ROS_INFO("Counter updating: %d", mCounter);
                    mpBurgerMotor->BurgerControl( mNoVelocity, aLeftOrRight * mBroadTurnVelocity );
                    mCounter = mCounter +1 ;
                }

                // continue following the wall using the FOLLOW_WALL state
                else
                {
                    ROS_INFO("[CWallFollower]: Obstacle cleared --> FOLLOW_WALL state.");
                    mCurrentState = FOLLOW_WALL;
                    mCounter = 0;
                }
            }
            break;

        // This state occurs when the robot has detected and navigated to an obstacle.
        // it Pauses to 'deal with the hazard'.
        // In extending the code, if an action was to be added for dealing with an 
        // hazard,  it would be placed here 
        case PAUSE:
        
            // pause for x number of seconds // function of 10 Hz as specified for CBurger [ ...rate(10) ]
            if (mCounter <= 10*3)
            {
                ROS_INFO("[CWallFollower]: PAUSE state. The Robot has detected a hazard");
                ROS_INFO("Counter updating: %d", mCounter);
                mpBurgerMotor->BurgerControl( mNoVelocity, mNoVelocity );
                mCounter = mCounter + 1;
            }

            //Once the pause is completed, continue to the INIT state, the hazard 
            // as been dealt with (using the Hazard handling logic)
            else
            {
                mStationPositionX = mCurrentX; 
                mStationPositionY = mCurrentY;
                mStationPositionW = mCurrentW;
                ROS_INFO("[CWallFollower]: --> CHazardHandler");
                mCurrentState = INIT;
                mCounter = 0;
            }
            break;

        // If the robot has returned home having dealt with all hazards, this state
        // stops the robot, as it has completed it mission 
        case STOP:
            ROS_INFO("[CWallFollower]: STOP state. The Robot has handled all hazards in the maze.");
            mpBurgerMotor->BurgerControl( mNoVelocity, mNoVelocity );
            mCurrentState = STOP;
            break;
    }
}

//--SetCurrentState
void CWallFollower::SetCurrentState( eWallFollowerState aRobotState )
{
    mCurrentState = aRobotState;
}

//--GetStationX
double CWallFollower::GetStationX()
{
    return mStationPositionX;
}

//--GetStationY
double CWallFollower::GetStationY()
{
    return mStationPositionY;
}

//--GetStationW
double CWallFollower::GetStationW()
{
    return mStationPositionW;
}

//--SetREstockCompleteCheck
void CWallFollower::SetRestockCompleteCheck( int aFlag )
{
    mRestockCompleteFlag = aFlag;
}

//--SetHazardHandled
void CWallFollower::SetHazardsHandled( int aSum )
{
    mHazardsHandled = aSum;
}