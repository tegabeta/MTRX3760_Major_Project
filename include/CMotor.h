// MTRX3760 Major Project
// CMotor .h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CMOTOR_H
#define _CMOTOR_H

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


//---CMotor Interface-------------------------------
// This class is able to publish information to the motors.
// Informaiton is passed into the class using the function
// BurgerControll(), which then assigns it to message commands
// and publishes these commands to the motors
class CMotor {

    public:
        CMotor(ros::NodeHandle& apNodeHandle);  //Contruction
        ~CMotor();                              //Destructor
        
        //BurgerControl is a function that is able to control the motors
        //by publishing commands to them. This function takes in a linear
        //velocity and angular velocity value, attributes them to a
        // message command and then publishes the command to the motor
        void BurgerControl( float aLinearVelocity, float aAngularVelocity );
        
    private:
        // Initalise a publisher
        ros::Publisher mVelocityPublisher;
};

#endif // _CMOTOR_H
