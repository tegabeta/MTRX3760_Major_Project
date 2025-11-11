// MTRX3760 Major Project
// CHazards .cpp file
// Author: Group "Our Group"

//--Include Header Files------------------
#include "CHazards.h"
#include "eRobotState.h"

//--Includes------------------------------
#include <iostream>

//--CHazards Implementation---------------

//--Constructor
CHazards::CHazards( eHazardState aHazardType, int aHazardSize )
{
    // std::cout << "CTor CHazards" << std::endl;

    // Set hazard type and size
    mHazardType = aHazardType;
    mHazardSize = aHazardSize;

}

//--Destructor
CHazards::~CHazards()
{
    //std::cout << "Dtor CWallFollower" << std::endl;
}

//--GetHazardType
eHazardState CHazards::GetHazardType()
{
    return mHazardType;
}

//--GetHazardSize
int CHazards::GetHazardSize()
{
    return mHazardSize;
}


