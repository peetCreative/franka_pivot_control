//
// Created by peetcreative on 17.02.21.
//
#include "FrankaPivotController.h"
#include "frankx/frankx.hpp"
#include "affx/affine.hpp"
#include "PivotControlMessages.h"

#include <thread>
#include <chrono>
using std::chrono_literals::operator""ms;
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace franka_pivot_control
{
    using Affine = affx::Affine;
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

    FrankaPivotController::FrankaPivotController(
            std::string robotHostname,
            double distanceEE2PP,
            double distanceEE2Tip,
            double dynamicRel,
            double cameraTilt):
            mMotionData(),
            mDistanceEE2PP(distanceEE2PP),
            mDistanceEE2Tip(distanceEE2Tip),
            mCameraTilt(cameraTilt)
    {
        try {
            mRobot.reset(new frankx::Robot(robotHostname));
            mRobot->automaticErrorRecovery();
        }
        catch (...)
        {
            std::cout << "Initializing Panda failed" << std::endl;
            mReady = false;
            return;
        }

        //alternatively
//        mRobot->velocity_rel = 0.01;
//        mRobot->acceleration_rel = 0.01;
//        mRobot->jerk_rel = 0.01;
        mRobot->setDynamicRel(dynamicRel);
        mRobot->setDefaultBehavior();

        // so the thread crashes and we can restart it properly
        mRobot->repeat_on_error = false;

////        GOTO custom start pose
//        std::array<double, 7> jointPos =
//                {1.54919, -0.539112, 0.108926, -2.1571, -0.0957537, 2.05686, 0.782756};
//        printJointPositions(jointPos);
////        jointPos.at(0) += 1.57079632679;
//        frankx::JointMotion turnright(jointPos);
//        mRobot->move(turnright);
//

        mMotionDataMutex = std::make_shared<std::mutex>();
        mMotionData.last_pose_lock = mMotionDataMutex;
        mReady = true;
    }

    FrankaPivotController::~FrankaPivotController()
    {
        mWaypointMotion.finish();
        mPivoting = false;
        while(mMoveThread.joinable() || mPivotThread.joinable())
        {
            if(mMoveThread.joinable())
                mMoveThread.join();
            if(mPivotThread.joinable())
                mPivotThread.join();
        }
    }
    //write helper function for factor
    void FrankaPivotController::move()
    {
        mIsRobotThreadRunning = true;
        std::cout << "start motion" << std::endl;
        try {
            if(!mRobot->move(mWaypointMotion, mMotionData))
            {
                mFrankaErrors.emplace_back("libfrankaerror");
                std::cout << "stop motion on libfranka error" << std::endl;
            }
        } catch (franka::CommandException exception)
        {
            mWaypointMotion.setNextWaypoints({});
        }

        if (mMotionData.didBreak())
            std::cout << "MotionData did Break" << std::endl;
        mIsRobotThreadRunning = false;
    }

    void FrankaPivotController::pivot()
    {
        // The DOFPose we currently think we want to go to, get's updated from newest
        DOFPose target;
        // If we move not directly our intermediate target
        DOFPose intermediateTarget;
        try {
            updateCurrentPoses();
            {
                //TODO: scoped mutex
                target = mCurrentDOFPose;
                intermediateTarget = mCurrentDOFPose;
            }
            double rot_epsilon = 0.1;
            double rot_epsilon_lim = rot_epsilon*0.1;
            double trans_z_epsilon = 0.01;
            double trans_z_epsilon_lim = trans_z_epsilon*0.1;

            while (true) {
                std::this_thread::sleep_for(1ms);
                //TODO: check if we are actually moving and not just blocked
                updateCurrentPoses();
                {
                    std::lock_guard<std::mutex> guard(mTargetCurrentMutex);
                    // when there is a new target DOFPose
                    // or when we approach an intermediate target
                    // calc and set new waypoint
                    if (!mPivoting)
                        break;
                    bool new_movement = false;
                    // setTargetDOFPose was called
                    if (mTargetDOFPose != target)
                    {
//                        std::cout << "PIVOTING " << "setTargetDOFPose was called" << std::endl;
                        new_movement = true;
                    }
                    // we are on a longer movement with intermediate targets
                    // and we approach a intermediateTarget
                    if (intermediateTarget != target)
                    {
                        if(mCurrentDOFPose.closeTo(intermediateTarget, rot_epsilon_lim,
                                                trans_z_epsilon_lim))
                        {
//                            std::cout << "PIVOTING " << "long movement reached intermediate" << std::endl;
                            new_movement = true;
                        }
                    }
                    if(!mIsRobotThreadRunning) {
//                        std::cout << "PIVOTING " << "Robot Stopped" << std::endl;
                        new_movement = true;
                    }
                    if (new_movement) {
//                        std::cout << "PIVOTING " << "new_movement" << std::endl;
                        target = mTargetDOFPose;
                        // replace the current intermediat target with the newest Target
                        intermediateTarget = mTargetDOFPose;
                        std::vector<frankx::Waypoint> waypoints{};

                        // check the diff_in angles so we are not moving to far in a straight line
                        double diff_pitch =
                                intermediateTarget.pitch - mCurrentDOFPose.pitch;
                        double diff_yaw =
                                intermediateTarget.yaw - mCurrentDOFPose.yaw;
                        double diff_roll =
                                intermediateTarget.roll - mCurrentDOFPose.roll;
                        double diff_trans_z =
                                intermediateTarget.transZ - mCurrentDOFPose.transZ;
                        double diff_trans_z_abs = std::abs(diff_trans_z);
                        double biggest_rot_diff_abs =
                                std::max(std::abs(diff_pitch),
                                         std::max(std::abs(diff_yaw),
                                                  std::abs(diff_roll)));
                        if (biggest_rot_diff_abs > rot_epsilon ||
                            diff_trans_z_abs > trans_z_epsilon) {
                            std::cout << "don not move to fast" << std::endl;
                            int steps = std::max(biggest_rot_diff_abs / rot_epsilon,
                                                 diff_trans_z_abs /
                                                 trans_z_epsilon);
                            intermediateTarget.pitch =
                                    mCurrentDOFPose.pitch + diff_pitch / steps;
                            intermediateTarget.yaw = mCurrentDOFPose.yaw + diff_yaw / steps;
                            intermediateTarget.roll =
                                    mCurrentDOFPose.roll + diff_roll / steps;
                            intermediateTarget.transZ =
                                    mCurrentDOFPose.transZ + diff_trans_z / steps;
                        }

                        //from DOFPose calculate Cartisian Affine
                        Affine targetAffine;
                        calcAffineFromDOFPose(intermediateTarget, targetAffine);

                        std::cout << "currentDOFPose:   " << mCurrentDOFPose.toString() << std::endl;
                        std::cout << "intermediatePose: " << intermediateTarget.toString() << std::endl;
                        std::cout << "targetPose: " << target.toString() << std::endl;
                        std::cout << "currentAffine:    " << mCurrentAffine.toString() << std::endl;
                        std::cout << "targetAffine:     " << targetAffine.toString() << std::endl;

                        mTargetWaypoint = movex::Waypoint(targetAffine);
                        mWaypointMotion.setNextWaypoint(mTargetWaypoint);

                        // check that our robot control loop is running, if not (re)start it.
                        if (!mIsRobotThreadRunning) {
                            mIsRobotThreadRunning = true;
                            std::cout << "start new thread" << std::endl;
                            mRobot->automaticErrorRecovery();
                            if (mMoveThread.joinable())
                                mMoveThread.join();
                            mMoveThread = std::thread(&FrankaPivotController::move,
                                                      this);
                        }
                    }
                }
            }
        }
        catch (...)
        {
            std::cout << "pivoting loop is broken" << std::endl;
        }
        mWaypointMotion.finish();
        while (mMoveThread.joinable())
            mMoveThread.join();
        {
            const std::lock_guard<std::mutex> lockGuard(mTargetCurrentMutex);
            mPivoting = false;
        }
    }

    bool FrankaPivotController::setSpeed(float dynamicRel)
    {
        //TODO: put this to ASSERT
        if(dynamicRel < 0 || dynamicRel >= 1 || mIsRobotThreadRunning)
            return false;
        try {
            mRobot->setDynamicRel(dynamicRel);
        }
        catch (...)
        {
            return false;
        }
        return true;
    }

    bool FrankaPivotController::startPivoting(
            pivot_control_messages::DOFPose startDOFPose)
    {
        const std::lock_guard<std::mutex> lockGuard(mTargetCurrentMutex);
        if (!mDOFBoundaries.poseInside(startDOFPose))
            return false;
        mWaypointMotion.return_when_finished = false;
        mCurrentDOFPose = startDOFPose;

        mCurrentAffine = mRobot->currentPose();
        std::cout << "mInitialEEAffine" << mCurrentAffine.toString() << std::endl;
        mCurrentDOFPose = {0,0,0,0};
        mDofPoseReady = true;
        mDofBoundariesReady = true;
//        auto rotation = Eigen::AngleAxisd(-cameraTilt, Eigen::Vector3d::UnitZ());
//        mYAxis = rotation * Eigen::Vector3d::UnitY();
//        mZAxis = rotation * Eigen::Vector3d::UnitY();
        mInitialPPAffine = mCurrentAffine;
        mInitialPPAffine.translate(Eigen::Vector3d(0,0,-mDistanceEE2PP));
        mTargetWaypoint = movex::Waypoint(mCurrentAffine);
        mWaypointMotion = movex::WaypointMotion({mTargetWaypoint}, false);

        mReady = true;
        if (!mPivoting)
        {
            if (mPivotThread.joinable())
                mPivotThread.join();
            mPivoting = true;
            mPivotThread = std::thread(&FrankaPivotController::pivot,
                                  this);
        }
        return true;
    }

    bool FrankaPivotController::stopPivoting()
    {
        {
            const std::lock_guard<std::mutex> lockGuard(mTargetCurrentMutex);
            mPivoting = false;
            mReady = false;
        }
        while(mPivotThread.joinable())
            mPivotThread.join();
        return true;
    }

    bool FrankaPivotController::moveCartesianZ(float z)
    {
        const std::lock_guard<std::mutex> lockGuard(mTargetCurrentMutex);
        if (mPivoting)
            return false;
        Affine targetAffine = mRobot->currentPose();
        targetAffine.translate({0,0, -z});
        movex::Waypoint targetWaypoint(targetAffine);
        movex::WaypointMotion targetWaypointMotion({targetWaypoint});
        return mRobot->move(targetWaypointMotion);
    }

    bool FrankaPivotController::moveJointSpace(std::array<double, 7> target)
    {
        const std::lock_guard<std::mutex> lockGuard(mTargetCurrentMutex);
        if (mPivoting)
            return false;
        return mRobot->move(movex::JointMotion(target));;
    }

    bool FrankaPivotController::setTargetDOFPose(DOFPose dofPose) {
        const std::lock_guard<std::mutex> lockGuard(mTargetCurrentMutex);
        if (!mPivoting || !mDOFBoundaries.poseInside(dofPose))
            return false;
//        updateCurrentPoses();
        mTargetDOFPose = dofPose;
        return true;
    }

    void FrankaPivotController::calcAffineFromDOFPose(
            DOFPose &dofPose,
            Affine &affine)
    {
        double radius = mDistanceEE2PP - dofPose.transZ;
        //from DOFPose calculate Cartisian Affine
        affine = mInitialPPAffine;
        // or do it the other way around
        affine.rotate(Eigen::AngleAxisd(
                mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());

        affine.rotate(Eigen::AngleAxisd(
                dofPose.pitch, Eigen::Vector3d::UnitX()).toRotationMatrix());
        affine.rotate(Eigen::AngleAxisd(
                dofPose.yaw, mYAxis).toRotationMatrix());
        affine.rotate(Eigen::AngleAxisd(
                dofPose.roll, mZAxis).toRotationMatrix());
        affine.rotate(Eigen::AngleAxisd(
                -mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());
        affine.translate(Eigen::Vector3d(0,0, radius));
    }

    void FrankaPivotController::calcDOFPoseFromAffine(
            Affine affine,
            DOFPose &dofPose, double &error)
    {
        Affine affine1 = affine;
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
        Affine zeroAffine = affine;
        zeroAffine.setX(0);
        zeroAffine.setY(0);
        zeroAffine.setZ(0);
        Affine a = mInitialPPAffine;
        a.setX(0);
        a.setY(0);
        a.setZ(0);
        a.rotate(Eigen::AngleAxisd(
                mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());
        zeroAffine.rotate(Eigen::AngleAxisd(
                mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());
        Affine diffAffine = a.inverse() * zeroAffine;
        Affine b =  diffAffine.inverse();
        auto diffRotation = b.rotation();
        double yaw1 = - std::asin(diffRotation(2,0));
        double cy1 = std::cos(yaw1);
        double yaw2 = M_PI - yaw1;
        double cy2 = std::cos(yaw2);
        if ( cy1 == 0 || cy2 == 0)
        {
            dofPose.pitch = 0;
            dofPose.yaw = -yaw1;
            dofPose.roll = 0;
        }
        else
        {
            double pitch = std::atan2(diffRotation(2,1)/cy1, diffRotation(2,2)/cy1);
            double roll = std::atan2(diffRotation(1,0)/cy1, diffRotation(0,0)/cy1);

            dofPose.pitch = -pitch;
            dofPose.yaw = -yaw1;
            dofPose.roll = -roll;

        }
    }

    DOFPose FrankaPivotController::updateCurrentPoses()
    {
        // We need to check first, if the robot is running
        if(mIsRobotThreadRunning)
        {
            const std::lock_guard<std::mutex> lock(*(mMotionDataMutex));
            mCurrentAffine = mMotionData.last_pose;
        }
        else
        {
            mCurrentAffine = mRobot->currentPose();
        }
        DOFPose dofPose;
        {
            const std::lock_guard<std::mutex> lock(mTargetCurrentMutex);
            calcDOFPoseFromAffine(mCurrentAffine, dofPose, mCurrentError);
            if(dofPose.closeTo(mTargetDOFPose, 0.001, 0.001))
                dofPose = mTargetDOFPose;
            mCurrentDOFPose = dofPose;
        }
        return dofPose;
    }

    bool FrankaPivotController::getCurrentDOFPose(DOFPose &pose)
    {
        if (!mDofPoseReady)
            return false;
        pose = updateCurrentPoses();
        return true;
    }

    bool FrankaPivotController::getDOFBoundaries(DOFBoundaries &boundaries)
    {
        if (!mDofBoundariesReady)
            return false;
        boundaries = mDOFBoundaries;
        return true;
    }

    bool FrankaPivotController::getCurrentTipPose(
            std::array<double, 3> &translation, std::array<double, 4> &rotation)
    {
        if (!mDofPoseReady)
            return false;
        {
            const std::lock_guard<std::mutex> lock(*(mMotionDataMutex));
            mCurrentAffine = mMotionData.last_pose;
        }
        Affine cameraTipPoseAffine = mCurrentAffine;
        cameraTipPoseAffine.translate({0,0,-mDistanceEE2Tip});
        Eigen::Vector3d translationE = cameraTipPoseAffine.translation();
        translation = {translationE.x(), translationE.y(), translationE.z()};
        rotation = {cameraTipPoseAffine.qW(),
                    cameraTipPoseAffine.qX(),
                    cameraTipPoseAffine.qY(),
                    cameraTipPoseAffine.qZ()};
        return true;
    }

    bool FrankaPivotController::getError(double &error)
    {
        if (!mDofPoseReady)
            return false;
        updateCurrentPoses();
        error = mCurrentError;
        return true;
    }

    bool FrankaPivotController::getFrankaError(std::string &frankaError)
    {
        if (!mFrankaErrors.empty())
        {
            frankaError = mFrankaErrors.front();
            mFrankaErrors.pop_front();
            return true;
        }
        else
            return false;
    }

    bool FrankaPivotController::isReady()
    {
        return mReady && PivotController::isReady();
    };
}
