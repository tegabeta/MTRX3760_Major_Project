// MTRX3760 Major Project
// CCamera .h file
// Author: Group "Our Group"

//---Definitions------------------------------------
#ifndef _CCAMERA_H
#define _CCAMERA_H

//---Include Header Files---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <eRobotState.h>
#include <CHazards.h>


//---Includes---------------------------------------
#include <algorithm>
#include <numeric>
#include <vector>

//---CCamera Interface------------------------------
// The class CCamera is able to take the information from the camera, and scan it. It looks for 
// colour the specified (different for each disaster type) within the camera input 
// and if the colour is picked up the class will change a bool value that the navigation class checks
// This class is constantly running.
class CCamera{

    public:
        CCamera(ros::NodeHandle& apNodeHandle, CHazards* apHazard );    // Constructor
        ~CCamera();                                                     // Destructor


        // The function ImageCallback converts the ROS image to OpenCV
        // format. It also checks if the camera is currently operational 
        // or not and then it calls the DetectColourMask fucntion. 
        void ImageCallback(const sensor_msgs::CompressedImage& apMsg);

        // This function gets the type of hazard been looked for from the 
        // class CHazard and sets the upper and lower bounds of the colour 
        // searched for by the camera accordinly.
        void SetColourMask();

        // This function DetectColourMask checks if the image is in the 
        // range of the mask values and sets the camera finish flag if any of 
        // those colours is detected. 
        void DetectColourMask( cv_bridge::CvImagePtr apCvImgPtr );

        // This function FinishCheck returns the camera finish flag. 
        bool FinishCheck();

        // This function ResetCameraFlag resets the flag for the camera
        void ResetCameraFlag( bool aFlag );
        
    private:

        // Finish bool value
        bool mFinish;

        // Has a relationship with the enum class eHarzardState
        eHazardState mHazardType;

        // Colour Mask values
        cv::Scalar mMaskLower1;
        cv::Scalar mMaskLower2;

        cv::Scalar mMaskUpper1;
        cv::Scalar mMaskUpper2;

        // Image subscriber
        ros::Subscriber mImageSubscriber;
    
        // Pointer to an instance of class CHazard set in the constructor
        CHazards* mpHazard;
};

#endif // _CCAMERA_H
