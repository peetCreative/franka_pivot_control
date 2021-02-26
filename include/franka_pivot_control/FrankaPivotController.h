//
// Created by peetcreative on 17.02.21.
//
#ifndef FRANKA_PIVOT_CONTROL_FRANKA_PIVOT_CONTROL_H
#define FRANKA_PIVOT_CONTROL_FRANKA_PIVOT_CONTROL_H

#include "FrankaPivotController.h"

#include "PivotControlMessages.h"

#include <memory>
#include <sstream>

using namespace pivot_control_messages;

namespace franka_pivot_control
{
    class FrankaPivotControllerIntern;
    class FrankaPivotController : public PivotController
    {
    private:
        FrankaPivotControllerIntern *mController = nullptr;
    public:
        FrankaPivotController(
                std::string robotHostname,
                float distanceEE2PP,
                float maxWaypointDist,
                float cameraTilt);
        bool setTargetDOFPose(
                DOFPose);
        bool getCurrentDOFPose(
                DOFPose &pose);
        bool getDOFBoundaries(
                DOFBoundaries &boundaries);
        bool isReady();

    };
}
#endif //FRANKA_PIVOT_CONTROL_FRANKA_PIVOT_CONTROL_H
