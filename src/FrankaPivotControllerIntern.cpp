//
// Created by peetcreative on 17.02.21.
//
#include "FrankaPivotControllerIntern.h"
#include "frankx/frankx.hpp"
#include "PivotControlMessages.h"

#include <thread>
#include <iostream>
#include <Eigen/Core>

namespace franka_pivot_control
{
    FrankaPivotControllerIntern::FrankaPivotControllerIntern(
            std::string robotHostname,
            float distanceEE2PP,
            float maxWaypointDist,
            float cameraTilt):
            mRobot(robotHostname)
    {
        try {
            mRobot.automaticErrorRecovery();
            mRobot.setDynamicRel(0.15);

            mCurrentAffine = mRobot.currentPose();
            mInitialEEAffine = mCurrentAffine;
            std::cout << "mInitialEEAffine" << mCurrentAffine.toString() << std::endl;
        }
        catch (...)
        {
            std::cout << "ERROR: Could not initialize sth here" << std::endl;
            return;
        }

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
        mPivotPoint = mInitialEEAffine.translation() - Eigen::Vector3d(0,0,-mDistanceEE2PP);
        mMaxWaypointDist = maxWaypointDist;
        //TODO: rotate with angles
        mYAxis = Eigen::Vector3d::UnitY();
        mZAxis = Eigen::Vector3d::UnitZ();
        mWaypointMotion = movex::WaypointMotion({}, false);
    }

    //write helper function for factor
    void FrankaPivotControllerIntern::move()
    {
        mIsThreadRunning = true;
        mRobot.move(mWaypointMotion);
        //TODO: catch errors and return false
        mIsThreadRunning = false;
    }

    bool FrankaPivotControllerIntern::setTargetDOFPose(DOFPose dofPose)
    {
        mCurrentAffine = mRobot.currentPose();
        if (mTargetDOFPose == dofPose)
            return true;
        std::cout << "setTargetDOFPose"<< std::endl
            << dofPose.toString() << std::endl;
        float radius = mDistanceEE2PP - dofPose.transZ;

        //from DOFPose calculate Cartisian Affine
        frankx::Affine targetAffine = mInitialEEAffine;
        // or do it the other way around
        targetAffine.translate(Eigen::Vector3d(0, 0, mDistanceEE2PP));
        targetAffine.rotate(Eigen::AngleAxisd(
                dofPose.pitch, Eigen::Vector3d::UnitX()).toRotationMatrix());
        targetAffine.rotate(Eigen::AngleAxisd(
                dofPose.yaw, Eigen::Vector3d::UnitY()).toRotationMatrix());
        targetAffine.rotate(Eigen::AngleAxisd(
                dofPose.roll, Eigen::Vector3d::UnitZ()).toRotationMatrix());
        targetAffine.translate(Eigen::Vector3d(0,0, -radius));

        // look at distance from current pose to and if over threshold seperate it in small steps
//        Eigen::Vector3d path = targetAffine.translation() - mCurrentAffine.translation();
//        float pathLength = path.norm();
//        int numWaypoints = std::ceil(pathLength / mMaxWaypointDist);
//        Eigen::Vector3d waypointDist = path/numWaypoints;
//
//
//        std::vector<frankx::Waypoint> waypoints;
        std::cout << "currentAffine" << mCurrentAffine.toString() << std::endl;
        std::cout << "targetAffine" << targetAffine.toString() << std::endl;

        mTargetWaypoint = movex::Waypoint(targetAffine);
        mWaypointMotion.setNextWaypoint(mTargetWaypoint);

        mIsThreadRunning = true;
        if(!mIsThreadRunning)
        {
            //move();
            mMoveThread = std::thread(&FrankaPivotControllerIntern::move, this);
        }
        mTargetDOFPose = dofPose;
        return true;
    }

    bool FrankaPivotControllerIntern::updateCurrentDOFPoseFromAffine()
    {
        //calculate point of shortest distance between -z axis of this transform and mPivotPoint
        //if distance over threshhold throw error

        // calculate trans_z by distance between pivot point and EndEffector
        // calculate pitch, yaw and roll
        //mCurrentAffine;

        return true;
    }

    bool FrankaPivotControllerIntern::getCurrentDOFPose(DOFPose &pose)
    {
        if (!mDofPoseReady)
            return false;
        //TODO: calculate from mCurrentAffine
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
