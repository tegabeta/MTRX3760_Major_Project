// MTRX3760 Major Project
// CMotor .cpp file
// Author: Group "Our Group"


//--Include Header Files---------------------------
#include <CMotor.h>

//--Includes---------------------------------------
#include <iostream>

//--CCmotor Implementation-------------------------

//--Contructor
CMotor::CMotor(ros::NodeHandle& apNodeHandle)
{
    // std::cout << "CTor CMotor" << std::endl;
    
    // Publisher for the velocity
    mVelocityPublisher = apNodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
}

//--Desctructor
CMotor::~CMotor()
{
    //std::cout << "Dtor CMotor" << std::endl;
}

//--BurgerControl
void CMotor::BurgerControl( float aLinearVelocity, float aAngularVelocity)
{
    // std::cout << "[BurgerControl]: LV and AV | " << aLinearVelocity << ", " << aAngularVelocity << std::endl;
    
    //Creates a command to be published to the motors
    geometry_msgs::Twist command;
    
    // Assigns the commands to linear and angular velocity information
    // pass from nagivation logic
    command.linear.x = aLinearVelocity;
    command.angular.z = aAngularVelocity;
    
    //Publishes information to motors
    mVelocityPublisher.publish(command);
}