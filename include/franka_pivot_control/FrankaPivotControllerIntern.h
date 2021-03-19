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
        DOFBoundaries mDOFBoundaries
                {
                        0.2, -0.5,
                        0.5,-0.5,
                        1.2, -1.2,
                        0.04, -0.02
                };
        double mCurrentError {0};
        double mDistanceEE2PP;
        double mDistanceEE2Tip;
        double mCameraTilt;
        Eigen::Vector3d mYAxis {Eigen::Vector3d::UnitY()};
        Eigen::Vector3d mZAxis {Eigen::Vector3d::UnitZ()};
        bool mReady {false};

        void calcAffineFromDOFPose( DOFPose &dofPose, frankx::Affine &affine );
        void calcDOFPoseFromAffine(
                frankx::Affine affine,
                DOFPose &dofPose, double &error);
        bool testCalc();
        bool updateCurrentDOFPoseFromAffine();
        void move();
    public:
        FrankaPivotControllerIntern(
                std::string robotHostname,
                double distanceEE2PP,
                double distanceEE2Tip,
                double dynamicRel,
                double cameraTilt);
        bool setTargetDOFPose(
                DOFPose);
        bool getCurrentDOFPose(
                DOFPose &laparoscopeDofPose);
        bool getDOFBoundaries(
                DOFBoundaries &laparoscopeDofBoundaries);
        bool getCurrentTipPose(
                std::array<double, 3> &translation, std::array<double, 4> &rotation);
        bool getError(double &error);
        bool isReady()
        {
            return mReady && PivotController::isReady();
        };
    };
}

#endif //FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H
