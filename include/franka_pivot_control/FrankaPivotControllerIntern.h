//
// Created by peetcreative on 17.02.21.
//

#ifndef FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H
#define FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H

#include "FrankaPivotController.h"

#include "PivotControlMessages.h"

#include "frankx/frankx.hpp"
#include <memory>
#include <Eigen/Core>

using namespace pivot_control_messages;

namespace franka_pivot_control
{
    // intern class
    class FrankaPivotControllerIntern : public pivot_control_messages::PivotController
    {
    private:
        frankx::Robot mRobot;
        std::thread mMoveThread;
        bool mIsThreadRunning {false};
        movex::WaypointMotion mWaypointMotion;
        movex::Waypoint mTargetWaypoint;
        std::shared_ptr<std::mutex> mMotionDataMutex;
        movex::MotionData mMotionData;
        frankx::Affine mCurrentAffine;
        frankx::Affine mInitialEEAffine;
        frankx::Affine mInitialPPAffine;
        frankx::Affine mInitialOrientAffine;
        DOFPose mTargetDOFPose;
        DOFPose mCurrentDOFPose;
        DOFBoundaries mDOFBoundaries;
        double mCurrentError {0};
        double mDistanceEE2PP;
        Eigen::Vector3d mYAxis;
        Eigen::Vector3d mZAxis;
        void calcAffineFromDOFPose( DOFPose &dofPose, frankx::Affine &affine );
        void calcDOFPoseFromAffine(
                frankx::Affine affine,
                DOFPose &dofPose, double &error);
        bool updateCurrentDOFPoseFromAffine();
        void move();
    public:
        FrankaPivotControllerIntern(
                std::string robotHostname,
                double distanceEE2PP,
                double dynamicRel,
                double cameraTilt);
        bool setTargetDOFPose(
                DOFPose);
        bool getCurrentDOFPose(
                DOFPose &laparoscopeDofPose);
        bool getDOFBoundaries(
                DOFBoundaries &laparoscopeDofBoundaries);
    };
}

#endif //FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H
