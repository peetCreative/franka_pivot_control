//
// Created by peetcreative on 17.02.21.
//
#include "FrankaPivotController.h"
#include "frankx/frankx.hpp"
#include "affx/affine.hpp"
#include "PivotControlMessages.h"

using std::chrono_literals::operator""ms;
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define ROT_EPSILON 0.1
#define TRANS_Z_EPSILON 0.1

namespace franka_pivot_control {
    using Affine = affx::Affine;

    void printJointPositions(std::array<double, 7> positions) {
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
            double cameraTilt) :
            mMotionData(),
            mDistanceEE2PP(distanceEE2PP),
            mDistanceEE2Tip(distanceEE2Tip),
            mCameraTilt(cameraTilt){
        try {
            mRobot.reset(new frankx::Robot(robotHostname));
            franka::RobotState state = mRobot->readOnce();
            if (state.robot_mode == franka::RobotMode::kReflex) {
                std::cout << "automatic Error Recovery" << std::endl;
                mRobot->automaticErrorRecovery();
            }
            state = mRobot->readOnce();
            if (state.robot_mode != franka::RobotMode::kIdle) {
                if (state.robot_mode == franka::RobotMode::kOther)
                    std::cout << "Robot in Other mode." << std::endl;
                if (state.robot_mode == franka::RobotMode::kMove)
                    std::cout << "Robot in Move mode." << std::endl;
                if (state.robot_mode == franka::RobotMode::kGuiding)
                    std::cout << "Robot in Guiding mode." << std::endl;
                if (state.robot_mode == franka::RobotMode::kReflex)
                    std::cout << "Robot in Reflex mode." << std::endl;
                if (state.robot_mode == franka::RobotMode::kUserStopped)
                    std::cout << "Robot in UserStopped mode." << std::endl;
                if (state.robot_mode == franka::RobotMode::kAutomaticErrorRecovery)
                    std::cout << "Robot in Automatic Recovery mode." << std::endl;
                mReady = false;
                return;
            } else {
                //alternatively
                mRobot->setDynamicRel(dynamicRel);
                mRobot->setDefaultBehavior();

                // so the thread crashes and we can restart it properly
                mRobot->repeat_on_error = false;

                mReady = true;
            }
            mMoveThread = std::thread(
                    &FrankaPivotController::moveThread,
                    this);
            mPivotThread = std::thread(
                    &FrankaPivotController::pivotThread,
                    this);
            mCheckPosePivotThread = std::thread(
                    &FrankaPivotController::checkPivotMovementThread,
                    this);
        }
        catch (franka::Exception exception) {
            std::cout << "Initializing Panda failed with: " << exception.what()
                      << std::endl;
            mReady = false;
            return;
        }

    }

    FrankaPivotController::~FrankaPivotController() {
        mWaypointMotion.finish();
        mQuitPivotThread.store(true);
        mNewMovementCV.notify_all();
        mPivotCV.notify_all();
        if (mPivotThread.joinable())
            mPivotThread.join();
        mCheckPosePivotCV.notify_all();
        if (mCheckPosePivotThread.joinable())
            mCheckPosePivotThread.join();
        mQuitMoveThread.store(true);
        mMoveCV.notify_one();
        if (mMoveThread.joinable())
            mMoveThread.join();
    }

    void FrankaPivotController::moveThread()
    {
        std::unique_lock lock(mMoveThreadMutex);
        std::cout << "start move thread" << std::endl;
        bool restart {false};
        while(true) {
            if (!restart)
                mMoveCV.wait(lock);
            if (mMoveing.load())
                move();
            restart = mMoveing;
            if (mQuitMoveThread.load()) {
                std::cout << "stop move thread" << std::endl;
                return;
            }
        }
    }

    //write helper function for factor
    void FrankaPivotController::move()
    {
        try {
            if (!mRobot->move(mWaypointMotion, mMotionData)) {
                franka::RobotState robotState = mRobot->readOnce();
                mFrankaErrors.emplace_back(
                        std::string(robotState.last_motion_errors));
                std::cout << "stop motion on libfranka error" << std::endl;
                mRobot->automaticErrorRecovery();
            }
        } catch (franka::CommandException exception) {
            std::cout << "franka error " << exception.what() << std::endl;
            mWaypointMotion.setNextWaypoints({});
        }

        if (mMotionData.didBreak())
            std::cout << "MotionData did Break" << std::endl;
    }

    void FrankaPivotController::pivotThread()
    {
        std::unique_lock lock(mPivotThreadMutex);
        std::cout << "start pivoting thread" << std::endl;
        while(true)
        {
            mPivotCV.wait(lock);
            if (mPivoting.load())
                pivot();
            mPivoting.store(false);
            if (mQuitPivotThread.load())
            {
                std::cout << "end pivoting thread" << std::endl;
                mPivoting.store(false);
                return;
            }
        }
    }

    void FrankaPivotController::checkPivotMovementThread() {
        const double rot_epsilon_lim = ROT_EPSILON * 0.1;
        const double trans_z_epsilon_lim = TRANS_Z_EPSILON * 0.1;

        std::unique_lock lock(mCheckPosePivotThreadMutex);
        std::cout << "start checking pivoting Movement thread" << std::endl;
        while (true)
        {
            mCheckPosePivotCV.wait(lock);
            if (mQuitPivotThread.load()) {
                std::cout << "stop checking pivoting Movement thread" << std::endl;
                return;
            }
            if (mPivoting.load())
            {
                while (true) {
                    std::this_thread::sleep_for(1ms);
                    //TODO: check if we are actually moving and not just blocked
                    if (!mPivoting.load()) {
                        std::cout << "end checking pivoting Movement"
                                  << std::endl;
                        if (mQuitPivotThread.load()) {
                            std::cout << "stop checking pivoting Movement thread" << std::endl;
                            return;
                        }
                        break;
                    }
                    // make sure we are just using read from WaypointMotion
                    franka::RobotState robotState;
                    mMotionData.getRobotState(robotState);
                    DOFPose currentDOFPose;
                    double error;
                    calcDOFPoseFromAffine(Affine(robotState.O_T_EE), currentDOFPose, error);

                    DOFPose intermediateTarget = mIntTargetDOFPose.load();
                    if (intermediateTarget != mTargetDOFPose.load()) {
//                        std::cout << "check: "
//                            << currentDOFPose.rollDiff(intermediateTarget) << " "
//                            << currentDOFPose.transZDiff(intermediateTarget) << std::endl;
                        if (currentDOFPose.closeTo(intermediateTarget,
                                                   rot_epsilon_lim,
                                                   trans_z_epsilon_lim)) {
                            mNewMovementCV.notify_one();
                        }
                    }
                }
            }
        }
    }
    void FrankaPivotController::pivot() {
        std::unique_lock lock(mNewMovementMutex);
        DOFPose target;
        DOFPose intermediateTarget;
        DOFPose currentDOFPose;
        while (true)
        {
            mNewMovementCV.wait(lock);
            if (!mPivoting.load())
                return;
            updateCurrentPoses(currentDOFPose);
            target = mTargetDOFPose.load();
            // replace the current intermediat target with the newest Target
            intermediateTarget = target;
            std::vector<frankx::Waypoint> waypoints{};

            // check the diff_in angles so we are not moving to far in a straight line
            double diff_pitch =
                    intermediateTarget.pitch -
                    currentDOFPose.pitch;
            double diff_yaw =
                    intermediateTarget.yaw - currentDOFPose.yaw;
            double diff_roll =
                    intermediateTarget.roll - currentDOFPose.roll;
            double diff_trans_z =
                    intermediateTarget.transZ -
                    currentDOFPose.transZ;
            double diff_trans_z_abs = std::abs(diff_trans_z);
            double biggest_rot_diff_abs =
                    std::max(std::abs(diff_pitch),
                             std::max(std::abs(diff_yaw),
                                      std::abs(diff_roll)));
            if (biggest_rot_diff_abs > ROT_EPSILON ||
                diff_trans_z_abs > TRANS_Z_EPSILON) {
                int steps = std::max(
                        biggest_rot_diff_abs / ROT_EPSILON,
                        diff_trans_z_abs /
                        TRANS_Z_EPSILON);
                intermediateTarget.pitch =
                        currentDOFPose.pitch + diff_pitch / steps;
                intermediateTarget.yaw =
                        currentDOFPose.yaw + diff_yaw / steps;
                intermediateTarget.roll =
                        currentDOFPose.roll + diff_roll / steps;
                intermediateTarget.transZ =
                        currentDOFPose.transZ +
                        diff_trans_z / steps;
            }

            mIntTargetDOFPose.store(intermediateTarget);
            //from DOFPose calculate Cartisian Affine
            Affine targetAffine;
            calcAffineFromDOFPose(intermediateTarget, targetAffine);

            std::cout << "currentDOFPose:   "
                      << currentDOFPose.toString() << std::endl;
            std::cout << "intermediatePose: "
                      << intermediateTarget.toString() << std::endl;
            std::cout << "targetPose: " << target.toString()
                      << std::endl;
            std::cout << "currentAffine:    "
                      << mCurrentAffine.toString() << std::endl;
            std::cout << "targetAffine:     "
                      << targetAffine.toString() << std::endl;

            mTargetWaypoint = movex::Waypoint(targetAffine);
            mWaypointMotion.setNextWaypoint(mTargetWaypoint);

            mMoveing.store(true);
            mMoveCV.notify_one();
        }
    }

    bool FrankaPivotController::setSpeed(float dynamicRel) {
        //TODO: put this to ASSERT
        if (dynamicRel < 0 || dynamicRel >= 1)
            return false;
        try {
            mRobot->setDynamicRel(dynamicRel);
        }
        catch (...) {
            return false;
        }
        return true;
    }

    //TODO: check that everything is thread safe
    bool FrankaPivotController::startPivoting(
            pivot_control_messages::DOFPose startDOFPose) {
        std::unique_lock<std::mutex> pivotLock(mPivotThreadMutex, std::defer_lock);
        if (pivotLock.try_lock())
        {
            {
                std::lock_guard<std::mutex> calcLock(mCalcMutex);
                if (!mDOFBoundaries.poseInside(startDOFPose))
                    return false;
                franka::RobotState robotState;
                if(!readState(robotState))
                    return false;
                mWaypointMotion.return_when_finished = false;
                mInitialPPAffine = Affine(robotState.O_T_EE);
                mInitialPPAffine.translate(Eigen::Vector3d(0, 0, -mDistanceEE2PP));
            }
            updateCurrentPoses();
            mTargetDOFPose = startDOFPose;
            mDofPoseReady = true;
            mDofBoundariesReady = true;
    //        auto rotation = Eigen::AngleAxisd(-cameraTilt, Eigen::Vector3d::UnitZ());
    //        mYAxis = rotation * Eigen::Vector3d::UnitY();
    //        mZAxis = rotation * Eigen::Vector3d::UnitY();
            mTargetWaypoint = movex::Waypoint(mCurrentAffine);
            mWaypointMotion = movex::WaypointMotion({mTargetWaypoint}, false);
            mPivoting.store(true);
            mPivotCV.notify_all();
            mCheckPosePivotCV.notify_all();
            std::cout << "notify booth pivot threads" << std::endl;
            return true;
        }
        else
        {
            std::cout << "seems that we are already pivoting" << std::endl;
            return false;
        }
    }

    bool FrankaPivotController::stopPivoting()
    {
        mPivoting.store(false);
        mReady = false;
        //TODO:maybe wait for answer that pivoting stopped
        return true;
    }

    bool FrankaPivotController::readState(franka::RobotState &robotState)
    {
        //TODO: or should we use mIsRobotThreadRunning
        if (mMoveing.load() && mMotionData.is_moving)
        {
            return mMotionData.getRobotState(robotState);
        }
        else
        {
            try {
                robotState = mRobot->readOnce();
            }
            catch (franka::Exception exception)
            {
                std::cout << "readState: can not read state" << exception.what() << std::endl;
                return false;
            }
            return true;
        }
    }

    bool FrankaPivotController::checkCanMove()
    {
        franka::RobotState robotState;
        if (!readState(robotState))
            return false;
        bool canMove {false};
        switch (robotState.robot_mode) {
            case franka::RobotMode::kIdle:
                canMove = true;
            case franka::RobotMode::kReflex:
            case franka::RobotMode::kAutomaticErrorRecovery:
                if (mRobot->hasErrors())
                {
                    mRobot->automaticErrorRecovery();
                    //TODO: this might not be correct
                    canMove = true;
                }
                else
                {
                    canMove = false;
                }

        }
        return canMove;
    }


    bool FrankaPivotController::moveCartesianZ(float z)
    {
        std::unique_lock<std::mutex> lock(mMoveThreadMutex, std::defer_lock);
        if (mPivoting.load() || !lock.try_lock())
            return false;
        Affine targetAffine = mRobot->currentPose();
        targetAffine.translate({0,0, -z});
        movex::Waypoint targetWaypoint(targetAffine);
        movex::WaypointMotion targetWaypointMotion({targetWaypoint});
        bool succ;
        if (mReady)
            succ = mRobot->move(targetWaypointMotion);
//        lock.release();
        return succ;
    }

    bool FrankaPivotController::moveJointSpace(std::array<double, 7> target)
    {
        std::unique_lock<std::mutex> lock(mMoveThreadMutex, std::defer_lock);
        if (mPivoting.load() || !lock.try_lock())
        {
//            lock.release();
            return false;
        }
        bool succ;
        if (mReady)
            succ = mRobot->move(movex::JointMotion(target));
//        lock.release();
//        mMoveThreadLock.release();
        mRobot->stop();
        return succ;
    }

    bool FrankaPivotController::setTargetDOFPose(DOFPose dofPose) {
        if (!mPivoting.load() || !mDOFBoundaries.poseInside(dofPose))
            return false;
        mIntTargetDOFPose.store(dofPose);
        mTargetDOFPose.store(dofPose);
        mNewMovementCV.notify_one();
        return true;
    }

    void FrankaPivotController::calcAffineFromDOFPose(
            const DOFPose &dofPose,
            Affine &affine)
    {
        std::lock_guard<std::mutex> lockGuard(mCalcMutex);
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
            const Affine &affine,
            DOFPose &dofPose, double &error)
    {
        std::lock_guard<std::mutex> lockGuard(mCalcMutex);

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

    bool FrankaPivotController::updateCurrentPoses()
    {
        franka::RobotState robotState;
        if(!readState(robotState))
            return false;
        Affine currentAffine = Affine(robotState.O_T_EE);
        mCurrentAffine = currentAffine;
        DOFPose targetDOFPose = mTargetDOFPose.load();
        DOFPose currentDofPose;
        double currentError;
        calcDOFPoseFromAffine(currentAffine, currentDofPose, currentError);
        if(currentDofPose.closeTo(targetDOFPose, 0.001, 0.001))
            currentDofPose = targetDOFPose;
        mCurrentDOFPose = currentDofPose;
        mCurrentError = currentError;
        return true;
    }

    bool FrankaPivotController::updateCurrentPoses(DOFPose &dofPose)
    {
        updateCurrentPoses();
        dofPose = mCurrentDOFPose;
    }

    bool FrankaPivotController::getCurrentDOFPose(DOFPose &pose)
    {
        if (!mDofPoseReady)
            return false;
        updateCurrentPoses(pose);
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
        updateCurrentPoses();
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
