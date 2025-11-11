// MTRX3760 Major Project
// CCamera .cpp file
// Author: Group "Our Group"


//--Include Header Files---------------------------
#include <CCamera.h>
#include <CHazards.h>
#include <cv_bridge/cv_bridge.h>

//--Includes---------------------------------------
#include <iostream>

//--CCamera Implementation-------------------------

//--Constructor
CCamera::CCamera(ros::NodeHandle& apNodeHandle, CHazards* apHazard)
{
    // std::cout << "CTor CCamera" << std::endl;
    
    // Set a pointer to the instance of the CHazard class passed through
    mpHazard = apHazard;
    
    // Run the SetColourMask function to initalise the type of disaster been
    // targetted
    SetColourMask();
    
    // Attribute a subscripter to the camera data
    mImageSubscriber = apNodeHandle.subscribe("/camera/image_raw/compressed", 10, &CCamera::ImageCallback, this);

    //Set bool value to 0 (the finish flag)
    mFinish = 0;
}

//--Destructor
CCamera::~CCamera()
{
    // std::cout << "DTor CCamera" << std::endl;
}

//--ImageCallBack
void CCamera::ImageCallback(const sensor_msgs::CompressedImage& apMsg) {

    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cvImgPtr;

    // Test to determine if camera signal is operational
    try 
    {
        cvImgPtr = cv_bridge::toCvCopy(apMsg, sensor_msgs::image_encodings::BGR8);
    } 

    catch (cv_bridge::Exception& cvException) 
    {
        ROS_ERROR("cv_bridge exception: %s", cvException.what());
        return;
    }

    // Call the DetectColourMask to test the signal from the camera
    DetectColourMask( cvImgPtr );

}

//--SetColourMask
void CCamera::SetColourMask()
{
    //Get the Hazard type specified in main by the user
    mHazardType = mpHazard->GetHazardType();

    //Set the boundries of the colour accordingly
    switch (mHazardType)
    {
        case FIRE:
            // Set HSV values for RED
            mMaskLower1 = cv::Scalar(0, 100, 100);
            mMaskLower2 = cv::Scalar(10, 255, 255);

            mMaskUpper1 = cv::Scalar(160, 100, 100);
            mMaskUpper2 = cv::Scalar(180, 255, 255);
            break;

        case FLOOD:
            // Set HSV values for BLUE
            mMaskLower1 = cv::Scalar(90, 50, 50);
            mMaskLower2 = cv::Scalar(130, 255, 255);

            mMaskUpper1 = cv::Scalar(90, 50, 50);
            mMaskUpper2 = cv::Scalar(130, 255, 255);
            break;

        case DEBRIS:
            // Set HSV values for GREEN
            mMaskLower1 = cv::Scalar(35, 50, 50);
            mMaskLower2 = cv::Scalar(85, 255, 255);

            mMaskUpper1 = cv::Scalar(35, 50, 50);
            mMaskUpper2 = cv::Scalar(85, 255, 255);
            break;
    }
}

//--DetectColourMask
void CCamera::DetectColourMask( cv_bridge::CvImagePtr apCvImgPtr )
{
    // Detect color 
    cv::Mat& image = apCvImgPtr->image;
    cv::Mat hsvImg;
    cv::cvtColor(image, hsvImg, cv::COLOR_BGR2HSV);

    // Determine if the colour is detected in each image processed
    cv::Mat maskLower;
    cv::Mat maskUpper;
    cv::inRange(hsvImg, mMaskLower1, mMaskLower2, maskLower);
    cv::inRange(hsvImg, mMaskUpper1, mMaskUpper2, maskUpper);

    cv::Mat mask;
    
    // The colour red (fire) is handled differently due to the way 
    // the HSV values work
    if (mHazardType == FIRE) 
    {
        mask = maskLower | maskUpper;
    }
    else
    {
        mask = maskLower;
    }

    // If colour detected, set mFinish to 1
    int colourCount = cv::countNonZero(mask);
    // ROS_INFO("[CCamera]: Detected colour pixels - %d", colourCount);

    if (colourCount > 30000)
    {
        // ROS_INFO("I CAN'T BELIEVE I FOUND SOME COLOUR HOLY SHIT THIS IS HAPPENING AHHHH");
        mFinish = 1;
    }

}

//--FinishCheck
bool CCamera::FinishCheck( )
{
    return mFinish;
}

//--ResetCameraFlag
void CCamera::ResetCameraFlag( bool aFlag )
{
    // Reset the flag according to the bool value input
    if (aFlag)
    {
        mFinish = 0;
    }
    else
    {
        mFinish = mFinish;
    }
}


