//
// Created by peetcreative on 17.02.21.
//

#ifndef FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H
#define FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H

#include "FrankaPivotControl.h"

#include "frankx/frankx.hpp"
#include <memory>
#include <Eigen/Core>

namespace franka_pivot_control
{
    // intern class
    class PivotControl
    {
    private:
        frankx::Robot mRobot;
        frankx::Affine mCurrentAffine;
        frankx::Affine mInitialEEAffine;
        DOFPose mCurrentDOFPose;
        DOFBoundaries mDOFBoundaries;
        float mDistanceEE2PP;
        Eigen::Vector3d mPivotPoint;
        float mMaxWaypointDist;
        Eigen::Vector3d mYAxis;
        Eigen::Vector3d mZAxis;
    public:
        PivotControl(
                std::string robotHostname,
                DOFBoundaries dofBoundaries,
                float distanceEE2PP,
                float maxWaypointDist,
                float cameraTilt);
        void setTargetDOFPose(DOFPose);
        DOFPose getCurrentDOFPose();
        DOFBoundaries getCurrentDOFBoundaries();
    };
}

#endif //FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H
