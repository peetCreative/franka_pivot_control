//
// Created by peetcreative on 17.02.21.
//
#include "FrankaPivotController.h"
#include "frankx/frankx.hpp"
#include "PivotControlMessages.h"

#include <thread>
#include <chrono>
using std::chrono_literals::operator""ms;
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

    FrankaPivotController::FrankaPivotController(
            std::string robotHostname,
            double distanceEE2PP,
            double distanceEE2Tip,
            double dynamicRel,
            double cameraTilt):
            mRobot(robotHostname),
            mMotionData(),
            mDistanceEE2PP(distanceEE2PP),
            mDistanceEE2Tip(distanceEE2Tip),
            mCameraTilt(cameraTilt)
    {
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
        mCurrentDOFPose = {0,0,0,0};
        mDofPoseReady = true;
        mDofBoundariesReady = true;
//        auto rotation = Eigen::AngleAxisd(-cameraTilt, Eigen::Vector3d::UnitZ());
//        mYAxis = rotation * Eigen::Vector3d::UnitY();
//        mZAxis = rotation * Eigen::Vector3d::UnitY();
        mInitialPPAffine = mInitialEEAffine;
        mInitialPPAffine.translate(Eigen::Vector3d(0,0,-mDistanceEE2PP));
        mInitialOrientAffine = mCurrentAffine;
        mInitialOrientAffine.set_x(0);
        mInitialOrientAffine.set_y(0);
        mInitialOrientAffine.set_z(0);
        if(!testCalc())
        {
            std::cout << "test calculation failed" << std::endl;
            return;
        }
        mTargetWaypoint = movex::Waypoint(mCurrentAffine);
        mWaypointMotion = movex::WaypointMotion({mTargetWaypoint}, false);
        mMotionDataMutex = std::make_shared<std::mutex>();
        mMotionData.last_pose_lock = mMotionDataMutex;
        mReady = true;
    }

    //write helper function for factor
    void FrankaPivotController::move()
    {
        mIsThreadRunning = true;
        std::cout << "start motion" << std::endl;
        try {
            if(!mRobot.move(mWaypointMotion, mMotionData))
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
        mIsThreadRunning = false;
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
            double rot_epsilon = 0.01;
            double rot_epsilon_lim = rot_epsilon*0.8;
            double trans_z_epsilon = 0.05;
            double trans_z_epsilon_lim = trans_z_epsilon*0.8;

            while (true) {
                std::this_thread::sleep_for(10ms);
                //TODO: check if we are actually moving and not just blocked
                {
                    std::lock_guard<std::mutex> guard(mTargetCurrentMutex);
                    // when there is a new target DOFPose
                    // or when we approach an intermediate target
                    // calc and set new waypoint
                    if (!mPivoting)
                        break;
                    if (mTargetDOFPose != target ||
                        (intermediateTarget != target &&
                         mCurrentDOFPose.closeTo(intermediateTarget, rot_epsilon_lim,
                                         trans_z_epsilon_lim))) {

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
                        frankx::Affine targetAffine;
                        calcAffineFromDOFPose(intermediateTarget, targetAffine);

                        //        std::cout << "currentAffine" << mCurrentAffine.toString() << std::endl;
                        //        std::cout << "targetAffine" << targetAffine.toString() << std::endl;

                        mTargetWaypoint = movex::Waypoint(targetAffine);
                        mWaypointMotion.setNextWaypoint(mTargetWaypoint);

                        // check that our robot control loop is running, if not (re)start it.
                        if (!mIsThreadRunning) {
                            mIsThreadRunning = true;
                            std::cout << "start new thread" << std::endl;
                            mRobot.automaticErrorRecovery();
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
        mPivoting = false;
    }

    bool FrankaPivotController::setSpeed(float dynamicRel)
    {
        if(dynamicRel > 0 && dynamicRel <= 1)
            return false;
        mRobot.setDynamicRel(dynamicRel);
        return true;
    }

    bool FrankaPivotController::startPivoting(
            pivot_control_messages::DOFPose startDOFPose)
    {
        if (!mDOFBoundaries.poseInside(startDOFPose))
            return false;
        mWaypointMotion.return_when_finished = false;
        mCurrentDOFPose = startDOFPose;
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
        mWaypointMotion.finish();
        mReady = false;
        mPivoting = false;
        return true;
    }

    bool FrankaPivotController::moveCartesianZ(float z)
    {
        if (mPivoting)
            return false;
        movex::Affine targetAffine = mCurrentAffine;
        targetAffine.translate({0,0,z});
        movex::Waypoint targetWaypoint(targetAffine);
        movex::WaypointMotion targetWaypointMotion({targetWaypoint});
        return mRobot.move(targetWaypointMotion);
    }

    bool FrankaPivotController::moveJointSpace(std::array<double, 7> target)
    {
        if (mPivoting)
            return false;
        return mRobot.move(movex::JointMotion(target));;
    }

    bool FrankaPivotController::setTargetDOFPose(DOFPose dofPose) {
        if (!mPivoting || !mDOFBoundaries.poseInside(dofPose))
            return false;
//        updateCurrentPoses();
        const std::lock_guard<std::mutex> lockGuard(mTargetCurrentMutex);
        mTargetDOFPose = dofPose;
        return true;
    }

    void FrankaPivotController::calcAffineFromDOFPose(
            DOFPose &dofPose,
            frankx::Affine &affine)
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
        frankx::Affine a = mInitialOrientAffine;
        a.rotate(Eigen::AngleAxisd(
                mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());
        zeroAffine.rotate(Eigen::AngleAxisd(
                mCameraTilt, Eigen::Vector3d::UnitX()).toRotationMatrix());
        frankx::Affine diffAffine = a.inverse() * zeroAffine;
        frankx::Affine b =  diffAffine.inverse();
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

    bool FrankaPivotController::testCalc()
    {
        bool succ = true;
        DOFPose initialDOFPose {0,0,0,0};
        frankx::Affine affineCalc;
        calcAffineFromDOFPose(initialDOFPose, affineCalc);
        std::cout << affineCalc.toString() << std::endl;
        if (!affineCalc.isApprox(mInitialEEAffine))
        {
            std::cout << "DOF to Affine doesn't work" << std::endl;
            succ = false;
        }
        else
            std::cout << "DOF to Affine works" << std::endl;
        std::vector<DOFPose> testDOFPoses {
                {-0.3,0,0, 0},
                {0,0.1,0, 0},
                {0,0,0.2, 0},
                {0,0,0, 0.2},
                {0,0.1,0, 0.1},
                {-1.0,1.0,0, 0.0},
                {0.2,0.1,0, 0.1},
        };
        double error;
        frankx::Affine testAffine;
        DOFPose resultDOFPose;
        for (auto it = testDOFPoses.begin(); it != testDOFPoses.end(); it++)
        {
            DOFPose testDOFPose = *it;
            error = 0;
            calcAffineFromDOFPose(testDOFPose, testAffine);
            calcDOFPoseFromAffine(testAffine, resultDOFPose, error);
            std::cout << "FROM:    " << testDOFPose.toString() << std::endl;
            std::cout << "To:      " << testAffine.toString() << std::endl;
            std::cout << "Back to: " << resultDOFPose.toString() << std::endl;
            if (!testDOFPose.closeTo(resultDOFPose, 0.0001, 0.0001))
            {
                std::cout << "calculation failed" << std::endl;
                succ = false;
            }
            std::cout << "------------------------" << std::endl;

        }
        return succ;
    }

    DOFPose FrankaPivotController::updateCurrentPoses()
    {
        {
            const std::lock_guard<std::mutex> lock(*(mMotionDataMutex));
            mCurrentAffine = mMotionData.last_pose;
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
        frankx::Affine cameraTipPoseAffine = mCurrentAffine;
        cameraTipPoseAffine.translate({0,0,-mDistanceEE2Tip});
        Eigen::Vector3d translationE = cameraTipPoseAffine.translation();
        translation = {translationE.x(), translationE.y(), translationE.z()};
        rotation = {cameraTipPoseAffine.q_w(),
                    cameraTipPoseAffine.q_x(),
                    cameraTipPoseAffine.q_y(),
                    cameraTipPoseAffine.q_z()};
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
