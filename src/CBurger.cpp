// MTRX3760 Project 1
// CBurger .cpp file
// Author: Group "Our Group"

//--Include Header Files---------------------------
#include <CBurger.h>
#include <CWallFollower.h>
#include <CHazardHandler.h>
#include <CHazards.h>
#include <CLidar.h>
#include <CMotor.h>
#include <CCamera.h>
#include <CCartographer.h>

//--Includes-------------------------
#include <iostream>

//--CBurger Implementation-------------------------

//--Constructor
CBurger::CBurger( ros::NodeHandle& apNodeHandle, CHazards* apHazard )
{
    // std::cout << "Ctor CBurger" << std::endl;

    // Initialise new components for CBurger to know
    mpBurgerCamera = new CCamera( apNodeHandle, apHazard );
    mpBurgerLidar = new CLidar( apNodeHandle );
    mpBurgerMotor = new CMotor( apNodeHandle );
    mpCartographer = new CCartographer( apNodeHandle );

    // Initialise a CWallFollower to also know these components
    mpWallFollower = new CWallFollower( mpBurgerCamera, mpBurgerMotor, mpBurgerLidar, mpCartographer );
    
    // Initialise a CHazardHandler to also know these components
    mpHazardHandler = new CHazardHandler( mpBurgerCamera, mpBurgerMotor, mpBurgerLidar, mpCartographer, apHazard );
}   

//--Destructor
CBurger::~CBurger()
{
    //std::cout << "Dtor CBurger" << std::endl;

    // Delete the instances of each class at the destruction of the 
    // of CBurger
    delete mpBurgerCamera;
    delete mpBurgerLidar;
    delete mpBurgerMotor;
    delete mpCartographer;
    delete mpWallFollower;
    delete mpHazardHandler;
    ros::shutdown(); 
}

//--RunNavigation
void CBurger::RunNavigation()
{
    //Ross communication rate
    ros::Rate rate(10);  // 10 Hz


    //Run while ros is ok 
    while (ros::ok()) 
    {
        
        ros::spinOnce();

        // Check if the home station is the same for both hazard handler and wall follower
        // if that tis not the case the hazard handler is reset
        if (mpWallFollower->GetStationX()!= mpHazardHandler->GetStationX())
        {
            mpHazardHandler->SetStationX(mpWallFollower->GetStationX());
            mpHazardHandler->SetStationY(mpWallFollower->GetStationY());
            mpHazardHandler->SetStationW(mpWallFollower->GetStationW());
        }

        // Set the Materials and hazards handled
        mpWallFollower->SetRestockCompleteCheck( mpHazardHandler->RestockCompleteCheck() );
        mpWallFollower->SetHazardsHandled( mpHazardHandler->GetHazardsHandled() );
        
        // Print information
        ROS_INFO("[CBurger]: Finish: %d, Restock: %d", mpBurgerCamera->FinishCheck(), mpHazardHandler->RestockCompleteCheck());
        ROS_INFO("[CBurger]: Get Resources: %d, Restock: %d", mpHazardHandler->GetResourcesCheck(), mpHazardHandler->RestockCompleteCheck());
        ROS_INFO("[CBurger]: WFF %i", mpHazardHandler->GetWallFollowerFlag());

        // Right wall follow if camera not detected
        if ( (mpBurgerCamera->FinishCheck() == 0) && (mpHazardHandler->RestockCompleteCheck() != 0) )
        {
            mpWallFollower->WallFollowLogic( mpHazardHandler->GetWallFollowerFlag() );
        }

        // If hazard resources not needed; handle hazard
        else if ( (mpHazardHandler->GetResourcesCheck() != 0) && (mpHazardHandler->RestockCompleteCheck() == 0))
        {

            mpWallFollower->WallFollowLogic( mpHazardHandler->GetWallFollowerFlag() );
        }

        // If hazard resources needed; go home, set the current state accordinly
        else if ((mpHazardHandler->GetWallFollowerFlag() > 0) && (mpHazardHandler->RestockCompleteCheck() == 1) )
        {   

            mpHazardHandler->HazardHandlerLogic( mpHazardHandler->GetWallFollowerFlag() );
            mpBurgerCamera->ResetCameraFlag( mpHazardHandler->GetHazardHandledFlag() );    
            mpWallFollower->SetCurrentState(INIT);
        }

        // Follow wall if nothing needed 
        else
        {
            mpWallFollower->WallFollowLogic( mpHazardHandler->GetWallFollowerFlag() );
        }

        rate.sleep();
    }
}