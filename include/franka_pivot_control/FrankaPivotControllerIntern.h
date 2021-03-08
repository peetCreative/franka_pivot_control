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
        DOFPose mTargetDOFPose;
        DOFPose mCurrentDOFPose;
        DOFBoundaries mDOFBoundaries;
        double mDistanceEE2PP;
        Eigen::Vector3d mPivotPoint;
        double mMaxWaypointDist;
        Eigen::Vector3d mYAxis;
        Eigen::Vector3d mZAxis;
        bool updateCurrentDOFPoseFromAffine();
        void move();
    public:
        FrankaPivotControllerIntern(
                std::string robotHostname,
                double distanceEE2PP,
                double maxWaypointDist,
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
