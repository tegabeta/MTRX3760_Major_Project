// MTRX3760 Major Project
// enum RobotState .h file
// Author: Group "Our Group"

//--Definitions------------------------------------
#ifndef _EROBOTSTATE_H
#define _EROBOTSTATE_H

//--Includes---------------------------------------
#include <iostream>

//--Define Enum------------------------------------
// This header file defines different cases used by the
// nagivation logic as an enum. The header file is shared
// will all other header files that require it.
 
enum eWallFollowerState     // enum defining the states of CWallFollwer 
{
    INIT,
    FIND_WALL,
    FOLLOW_WALL,
    ROTATE,
    AVOID_OBSTACLE,
    PAUSE,
    STOP
};

enum eHazardHandlerState    // enum defining the states of CHazardHandler 
{
    START,
    FIND_HAZARD,
    FACE_HAZARD,
    HANDLER_HAZARD,
    GO_BACK,
    RESOURCE_CHECK,
    END
};

enum eHazardState           // enum defining the states of CHazards 
{
    FIRE,
    FLOOD,
    DEBRIS
};
#endif // _EROBOTSTATE_H
