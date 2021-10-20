//
// Created by peetcreative on 17.02.21.
//

#ifndef FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H
#define FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H

#include "frankx/frankx.hpp"
#include "PivotControlMessages.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <memory>
#include <deque>
#include <iostream>
#include <mutex>

using namespace pivot_control_messages;

namespace franka_pivot_control
{
    /*! \brief This class implements the interface
     * [PivotController](https://peetcreative.github.io/franka_pivot_control/classpivot__control__messages_1_1_pivot_controller.html)
     * by the robot
     */
    class FrankaPivotController : public pivot_control_messages::PivotController
    {
    protected:
        std::unique_ptr<frankx::Robot> mRobot;
        std::thread mMoveThread;
        std::thread mPivotThread;
        bool mIsRobotThreadRunning {false};
        bool mPivoting {false};
        movex::WaypointMotion mWaypointMotion;
        movex::Waypoint mTargetWaypoint;
        std::shared_ptr<std::mutex> mMotionDataMutex;
        movex::MotionData mMotionData;
        frankx::Affine mCurrentAffine;
        //! just to evaluate the calcXtoY functions
        frankx::Affine mInitialPPAffine;
        DOFPose mTargetDOFPose;
        DOFPose mCurrentDOFPose;
        std::mutex  mTargetCurrentMutex;
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
        std::deque<std::string> mFrankaErrors {};

        //! \brief Test function for calcAffineFromDOFPose
        bool testCalc(movex::Affine initialEEAffine);
        //! \brief Function to be run in a seperate thread to run the command loop
        DOFPose updateCurrentPoses();
        //! \brief Function to be run in a seperate thread to run the command loop
        void move();
        void pivot();
    public:
        /*! \brief Constructor of the Interface class
         *
         * @param robotHostname hostname/IP of the robot
         * @param distanceEE2PP distance from Endeffector (Robot flange) to PivotPoint
         * @param distanceEE2Tip distance from Endeffector (Robot flange) to Tip of the tool (Camera in _EndoMersion_)
         * @param dynamicRel dynamic relation to be configured to franka
         * @param cameraTilt angle of the camera tilt, atm not used
         */
        FrankaPivotController(
                std::string robotHostname,
                double distanceEE2PP,
                double distanceEE2Tip,
                double dynamicRel,
                double cameraTilt);
        //! \brief Set speed Value between 0 and 1
        /*! This function set the speed of the Franka Emika Robot
         *
         * @param dynamicRel Value between 0 (min) and 1 (max)
         * @return success
         */
        bool setSpeed(float dynamicRel);
        //! \brief Start Pivoting from this pose the robot is currently in.
        /*! this will start the run-function thread.
         *
         * @param startDOFPose DOFPose to start from
         * @return Success, if the robot is used return false
         */
        bool startPivoting(
                pivot_control_messages::DOFPose startDOFPose = {0, 0, 0, 0});
        //! \brief Stop Pivoting release the robot
        /*! stops the run-function thread
         *
         * @return success
         */
        bool stopPivoting();
        //! \brief Function starting a joint movement
        /*! if the robot is not pivoting (stopPivoting) the robot moves to this joint Position
         *
         * @param target target angles of the seven joints of Panda robot
         * @return success
         */
        bool moveJointSpace(std::array<double, 7> target);
        //! \brief Function starting a cartesian movement along the z-Axis of the flange
        /*! if the robot is not pivoting (stopPivoting) the robot moves (cartesian) along the z-Axis of the flange
         * This is to be used to move the tool out.
         *
         * @param z target distance to move
         * @return success
         */
        bool moveCartesianZ(float z);
        //! \brief implements pivot_control_messages::PivotController::setTargetDOFPose
        //! See [PivotController::setTargetDOFPose](https://peetcreative.github.io/franka_pivot_control/classpivot__control__messages_1_1_pivot_controller.html)
        bool setTargetDOFPose(
                DOFPose) override;
        //! \brief implements pivot_control_messages::PivotController::getCurrentDOFPose
        //! See [PivotController::getCurrentDOFPose](https://peetcreative.github.io/franka_pivot_control/classpivot__control__messages_1_1_pivot_controller.html)
        bool getCurrentDOFPose(
                DOFPose &laparoscopeDofPose) override;
        //! \brief implements pivot_control_messages::PivotController::getDOFBoundaries
        //! See [PivotController::getDOFBoundaries](https://peetcreative.github.io/franka_pivot_control/classpivot__control__messages_1_1_pivot_controller.html)
        bool getDOFBoundaries(
                DOFBoundaries &laparoscopeDofBoundaries) override;
        //! \brief get the transformation of the tooltip
        /*! This function gets the current transformation
         *  of the tooltip (Camera for _EndoMersion_)
         *  relative to origin of the robot.
         */
        bool getCurrentTipPose(
                std::array<double, 3> &translation, std::array<double, 4> &rotation);
        //! \brief get calculated distance of the tool-Axis to the pivot point
        /*! This function gets the distance,
         * when projecting the pivot to the tool-Axis.
         * Optimally this should be tend towards zero,
         * the tool going directly through the entry hole.
         *
         *  \param error [out] distance error value in Meter
         *  \return success
         */
        bool getError(double &error);
        //! \brief pop the first element from the  MessageQueue of libfranka errors
        /*! While pivoting the libfranka sometimes stops
         * and the controller restarts the movements.
         * The according messages are stored in a queue.
         *
         * @param frankaError String describing the error. generated by libfranka
         * @return true if there is a message
         */
        bool getFrankaError(std::string &frankaError);
        //! \brief indicate if the controller is ready to pivot
        //! See [PivotController::isReady](https://peetcreative.github.io/franka_pivot_control/classpivot__control__messages_1_1_pivot_controller.html)
        bool isReady();

        //! \brief Function to calculate the target Affine pose from the DOFPose
        void calcAffineFromDOFPose( DOFPose &dofPose, frankx::Affine &affine );
        //! \brief Function to calculate the DOFPose from the affine
        void calcDOFPoseFromAffine(
                frankx::Affine affine,
                DOFPose &dofPose, double &error);

    };
}

#endif //FRANKA_PIVOT_CONTROL_PIVOT_CONTROL_H
