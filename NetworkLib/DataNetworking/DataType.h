#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <iostream>

namespace robot_libs
{
    namespace networking 
    {
        enum RobotDataType : int
        {
            ROBOT_STATE      = 1,
            HEIGHT_MAP       = 2,
            CUSTOM_GAIT_DATA = 3,
            HMI_DATA         = 4
        };

    }
}

#endif
