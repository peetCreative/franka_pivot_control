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
                double distanceEE2PP,
                double distanceEE2Tip,
                double dynamicRel,
                double cameraTilt);
        bool setTargetDOFPose(
                DOFPose);
        bool getCurrentDOFPose(
                DOFPose &pose);
        bool getDOFBoundaries(
                DOFBoundaries &boundaries);
        bool getCurrentTipPose(
                std::array<double, 3> &translation, std::array<double, 4> &rotation);
        bool getError(double &error);
        bool isReady();

    };
}
#endif //FRANKA_PIVOT_CONTROL_FRANKA_PIVOT_CONTROL_H
