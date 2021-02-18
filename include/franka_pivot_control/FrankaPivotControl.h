//
// Created by peetcreative on 17.02.21.
//

#ifndef FRANKA_PIVOT_CONTROL_FRANKA_PIVOT_CONTROL_H
#define FRANKA_PIVOT_CONTROL_FRANKA_PIVOT_CONTROL_H

#include <memory>

namespace franka_pivot_control
{
    struct DOFPose
    {
        float pitch = 0;
        float yaw = 0;
        float roll = 0;
        float transZ = 0;
    };
    struct DOFBoundaries
    {
        float pitch = 0;
        float yaw = 0;
        float roll = 0;
        float transZ = 0;
    };
    class PivotControl;
    class FrankaPivotControl
    {
    private:
        std::unique_ptr<PivotControl> mController = nullptr;
    public:
        FrankaPivotControl(
                std::string robotHostname,
                DOFBoundaries boundaries,
                float distanceEE2PP,
                float maxWaypointDist,
                float cameraTilt);
        void setTargetDOFPose(DOFPose);
        DOFPose getCurrentDOFPose();
        DOFBoundaries getCurrentDOFBoundaries();
    };
}
#endif //FRANKA_PIVOT_CONTROL_FRANKA_PIVOT_CONTROL_H