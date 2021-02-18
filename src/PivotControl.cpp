//
// Created by peetcreative on 17.02.21.
//
#include "PivotControl.h"
#include "FrankaPivotControl.h"
#include "frankx/frankx.hpp"

#include <iostream>
#include <Eigen/Core>

namespace franka_pivot_control
{
    PivotControl::PivotControl(
            std::string robotHostname,
            DOFBoundaries dofBoundaries,
            float distanceEE2PP,
            float maxWaypointDist,
            float cameraTilt):
            mRobot(robotHostname)
    {
        //TODO:findout what this is doin
        mRobot.automaticErrorRecovery();
        //TODO:findout what this is doin
        mRobot.setDynamicRel(0.15);

        mCurrentAffine = mRobot.currentPose();
        mInitialEEAffine = mCurrentAffine;
        std::cout << "mInitialEEAffine" << mCurrentAffine.toString() << std::endl;
        mCurrentDOFPose = {0,0,0,0};
        mDOFBoundaries = dofBoundaries;
        mDistanceEE2PP = distanceEE2PP;
        mPivotPoint = mInitialEEAffine.translation() - Eigen::Vector3d(0,0,-mDistanceEE2PP);
        mMaxWaypointDist = maxWaypointDist;
        //TODO: rotate with angles
        mYAxis = Eigen::Vector3d::UnitY();
        mZAxis = Eigen::Vector3d::UnitZ();
    }

    //write helper function for factor

    void PivotControl::setTargetDOFPose(DOFPose dofPose)
    {
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
        mCurrentAffine = mRobot.currentPose();

        // look at distance from current pose to and if over threshold seperate it in small steps
        Eigen::Vector3d path = targetAffine.translation() - mCurrentAffine.translation();
        float pathLength = path.norm();
        int numWaypoints = std::ceil(pathLength / mMaxWaypointDist);
        Eigen::Vector3d waypointDist = path/numWaypoints;
        std::vector<frankx::Waypoint> waypoints;
        frankx::Waypoint waypoint(targetAffine);
        waypoints.push_back(waypoint);
        std::cout << "currentAffine" << mCurrentAffine.toString() << std::endl;
        std::cout << "targetAffine" << targetAffine.toString() << std::endl;

        frankx::WaypointMotion waypointMotion(waypoints);
        //TODO: check move is blocking
        mRobot.move(waypointMotion);
        mCurrentDOFPose = dofPose;
    }

    DOFPose PivotControl::getCurrentDOFPose()
    {
        //TODO: calculate from mCurrentAffine
        return mCurrentDOFPose;
    }

    DOFBoundaries PivotControl::getCurrentDOFBoundaries()
    {
        return mDOFBoundaries;
    }
}
