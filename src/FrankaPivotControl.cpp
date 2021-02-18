//
// Created by peetcreative on 17.02.21.
//
#include "FrankaPivotControl.h"
#include "PivotControl.h"
#include <memory>
namespace franka_pivot_control
{
    FrankaPivotControl::FrankaPivotControl(
            std::string robotHostname,
            DOFBoundaries dofBoundaries,
            float distanceEE2PP,
            float maxWaypointDist,
            float cameraTilt)
    {
        mController = std::make_unique<PivotControl>(
                robotHostname, dofBoundaries,
                distanceEE2PP, maxWaypointDist, cameraTilt);
    }

    void FrankaPivotControl::setTargetDOFPose(DOFPose dofPose)
    {
        mController->setTargetDOFPose(dofPose);
    }

    DOFPose FrankaPivotControl::getCurrentDOFPose()
    {
        return mController->getCurrentDOFPose();
    }

    DOFBoundaries FrankaPivotControl::getCurrentDOFBoundaries()
    {
        return mController->getCurrentDOFBoundaries();
    }
}
