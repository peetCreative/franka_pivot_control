//
// Created by peetcreative on 17.02.21.
//
#include "FrankaPivotControllerIntern.h"
#include "frankx/frankx.hpp"
#include "PivotControlMessages.h"

#include <thread>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace franka_pivot_control
{
    void printJointPositions(std::array<double,7> positions)
    {
        std::cout << "["
            << positions.at(0) << " "
            << positions.at(1) << " "
            << positions.at(2) << " "
            << positions.at(3) << " "
            << positions.at(4) << " "
            << positions.at(5) << " "
            << positions.at(6) << "]" << std::endl;
    }

    FrankaPivotControllerIntern::FrankaPivotControllerIntern(
            std::string robotHostname,
            double distanceEE2PP,
            double dynamicRel,
            double cameraTilt):
            mRobot(robotHostname),
            mMotionData()
    {
        std::cout << "Initializing Panda" << std::endl;
        try {
            mRobot.automaticErrorRecovery();
        }
        catch (...)
        {
            std::cout << "Initializing Panda failed" << std::endl;
            return;
        }

        //alternatively
//        mRobot.velocity_rel = 0.01;
//        mRobot.acceleration_rel = 0.01;
//        mRobot.jerk_rel = 0.01;
        mRobot.setDynamicRel(dynamicRel);
        mRobot.setDefaultBehavior();

        // so the thread crashes and we can restart it properly
        mRobot.repeat_on_error = false;

////        GOTO custom start pose
//        std::array<double, 7> jointPos =
//                {1.54919, -0.539112, 0.108926, -2.1571, -0.0957537, 2.05686, 0.782756};
//        printJointPositions(jointPos);
////        jointPos.at(0) += 1.57079632679;
//        frankx::JointMotion turnright(jointPos);
//        mRobot.move(turnright);
//

        mCurrentAffine = mRobot.currentPose();
        mInitialEEAffine = mCurrentAffine;
        std::cout << "mInitialEEAffine" << mCurrentAffine.toString() << std::endl;
        //TODO: set mCurrentDOFPoseReady mDOFBoundariesReady ready
        mCurrentDOFPose = {0,0,0,0};
        mDofPoseReady = true;
        //TODO: calculate limits from the joints limits
        mDOFBoundaries =
                {
                0.5, -0.5,
                1.2,-1.2,
                1.2, -1.2,
                0.32, 0
                };
        mDofBoundariesReady = true;
        mDistanceEE2PP = distanceEE2PP;
        mCameraTilt = cameraTilt;
        mInitialPPAffine = mInitialEEAffine;
        mInitialPPAffine.translate(Eigen::Vector3d(0,0,-mDistanceEE2PP));
        mInitialOrientAffine = mCurrentAffine;
        mInitialOrientAffine.set_x(0);
        mInitialOrientAffine.set_y(0);
        mInitialOrientAffine.set_z(0);
        //TODO: rotate with angles
        mYAxis = Eigen::Vector3d::UnitY();
        mZAxis = Eigen::Vector3d::UnitZ();
        mTargetWaypoint = movex::Waypoint(mCurrentAffine);
        mWaypointMotion = movex::WaypointMotion({mTargetWaypoint}, false);
        mMotionDataMutex = std::make_shared<std::mutex>();
        mMotionData.last_pose_lock = mMotionDataMutex;
        mMoveThread = std::thread(&FrankaPivotControllerIntern::move, this);
    }

    //write helper function for factor
    void FrankaPivotControllerIntern::move()
    {
        mIsThreadRunning = true;
        std::cout << "start motion" << std::endl;
        if(!mRobot.move(mWaypointMotion, mMotionData))
            std::cout << "stop motion on libfranka error" << std::endl;

        if (mMotionData.didBreak())
            std::cout << "MotionData did Break" << std::endl;
        //TODO: catch errors and return false
        mIsThreadRunning = false;
    }

    bool FrankaPivotControllerIntern::setTargetDOFPose(DOFPose dofPose)
    {
        //read current Affine from MotionData
        {
            const std::lock_guard<std::mutex> lock(*(mMotionDataMutex));
            mCurrentAffine = mMotionData.last_pose;
        }
        if (mTargetDOFPose == dofPose)
        {
            std::cout << "nothing to do"<< std::endl;
            return true;
        }
        std::cout << "setTargetDOFPose"<< std::endl
            << dofPose.toString() << std::endl;
        double radius = mDistanceEE2PP - dofPose.transZ;

        //from DOFPose calculate Cartisian Affine
        frankx::Affine targetAffine;
        calcAffineFromDOFPose(dofPose, targetAffine);

        // look at distance from current pose to and if over threshold seperate it in small steps
//        Eigen::Vector3d path = targetAffine.translation() - mCurrentAffine.translation();
//        double pathLength = path.norm();
//        int numWaypoints = std::ceil(pathLength / mMaxWaypointDist);
//        Eigen::Vector3d waypointDist = path/numWaypoints;
//
//
//        std::vector<frankx::Waypoint> waypoints;
        std::cout << "currentAffine" << mCurrentAffine.toString() << std::endl;
        std::cout << "targetAffine" << targetAffine.toString() << std::endl;

        mTargetWaypoint = movex::Waypoint(targetAffine);
        mWaypointMotion.setNextWaypoint(mTargetWaypoint);

        if (!mIsThreadRunning)
        {
            mIsThreadRunning = true;
            std::cout << "start new thread" << std::endl;
            mRobot.automaticErrorRecovery();
            mMoveThread.join();
            mMoveThread = std::thread(&FrankaPivotControllerIntern::move, this);
        }
        mTargetDOFPose = dofPose;
        return true;
    }

    void FrankaPivotControllerIntern::calcAffineFromDOFPose(
            DOFPose &dofPose,
            frankx::Affine &affine)
    {
        double radius = mDistanceEE2PP - dofPose.transZ;
        //from DOFPose calculate Cartisian Affine
        affine = mInitialPPAffine;
        // or do it the other way around
        affine.rotate(Eigen::AngleAxisd(
                -mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());
        affine.rotate(Eigen::AngleAxisd(
                dofPose.roll, mZAxis).toRotationMatrix());
        affine.rotate(Eigen::AngleAxisd(
                dofPose.yaw, mYAxis).toRotationMatrix());
        affine.rotate(Eigen::AngleAxisd(
                dofPose.pitch, Eigen::Vector3d::UnitX()).toRotationMatrix());
        affine.translate(Eigen::Vector3d(0,0, radius));
    }

    void FrankaPivotControllerIntern::calcDOFPoseFromAffine(
            frankx::Affine affine,
            DOFPose &dofPose, double &error)
    {
        frankx::Affine affine1 = affine;
        affine1.translate(Eigen::Vector3d::UnitZ());
        Eigen::ParametrizedLine<double, 3> line
        = Eigen::ParametrizedLine<double, 3>::Through(
                affine.translation(),
                affine1.translation()
                );
        Eigen::Vector3d closestPoint =
                line.projection(mInitialPPAffine.translation());
        double distanceToClosestPoint =
                (affine.translation() - closestPoint).norm();
        dofPose.transZ =
                mDistanceEE2PP - distanceToClosestPoint;
        error = (mInitialPPAffine.translation() - closestPoint).norm();
        frankx::Affine zeroAffine = affine;
        zeroAffine.set_x(0);
        zeroAffine.set_y(0);
        zeroAffine.set_z(0);
        // do we have to apply from left
        frankx::Affine diffAffine = mInitialOrientAffine.inverse() * zeroAffine;
        diffAffine.rotate(Eigen::AngleAxisd(
                mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());
        Eigen::Vector3d angles = diffAffine.angles();
        dofPose.pitch = angles.z();
        dofPose.yaw = angles.y();
        dofPose.roll = angles.x();
    }

    bool FrankaPivotControllerIntern::updateCurrentDOFPoseFromAffine()
    {
        {
            const std::lock_guard<std::mutex> lock(*(mMotionDataMutex));
            mCurrentAffine = mMotionData.last_pose;
        }
        calcDOFPoseFromAffine(mCurrentAffine, mCurrentDOFPose, mCurrentError);
        return true;
    }

    bool FrankaPivotControllerIntern::getCurrentDOFPose(DOFPose &pose)
    {
        if (!mDofPoseReady)
            return false;
        updateCurrentDOFPoseFromAffine();
        pose = mCurrentDOFPose;
        return true;
    }

    bool FrankaPivotControllerIntern::getDOFBoundaries(DOFBoundaries &boundaries)
    {
        if (!mDofBoundariesReady)
            return false;
        boundaries = mDOFBoundaries;
        return true;
    }
}
