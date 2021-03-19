//
// Created by peetcreative on 17.02.21.
//
#include "FrankaPivotController.h"
#include "FrankaPivotControllerIntern.h"
#include <memory>
namespace franka_pivot_control
{
    FrankaPivotController::FrankaPivotController(
            std::string robotHostname,
            double distanceEE2PP,
            double distanceEE2Tip,
            double dynamicRel,
            double cameraTilt)
    {
        mController = new FrankaPivotControllerIntern(
                robotHostname, distanceEE2PP, distanceEE2Tip,
                dynamicRel, cameraTilt);
    }

    bool FrankaPivotController::setTargetDOFPose(DOFPose dofPose)
    {
        return mController->setTargetDOFPose(dofPose);
    }

    bool FrankaPivotController::getCurrentDOFPose(DOFPose &pose)
    {
        return mController->getCurrentDOFPose(pose);
    }

    bool FrankaPivotController::getDOFBoundaries(DOFBoundaries &boundaries)
    {
        return mController->getDOFBoundaries(boundaries);
    }

    bool FrankaPivotController::getCurrentTipPose(
            std::array<double, 3> &translation, std::array<double, 4> &rotation)
    {
        return mController->getCurrentTipPose(translation, rotation);
    }

    bool FrankaPivotController::getError(double &error)
    {
        return mController->getError(error);
    }

    bool FrankaPivotController::isReady()
    {
        return mController->isReady();
    }
}
