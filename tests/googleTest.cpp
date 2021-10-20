#include "FrankaPivotController.h"
#include "PivotControlMessages.h"
#include "frankx/frankx.hpp"

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
using std::chrono_literals::operator""s;

constexpr float distanceEE2PP = 0.2;
constexpr float dynamicRel = 0.1;
constexpr float cameraTilt = -0.52359;

constexpr double epsilon = 0.001;

void testAffineApprox(frankx::Affine a, frankx::Affine b)
{
    EXPECT_LT(std::abs(a.x() - b.x()) , epsilon);
    EXPECT_LT(std::abs(a.y() - b.y()) , epsilon);
    EXPECT_LT(std::abs(a.z() - b.z()) , epsilon);
    EXPECT_LT(std::abs(a.q_w() - b.q_w()) , epsilon);
    EXPECT_LT(std::abs(a.q_x() - b.q_x()) , epsilon);
    EXPECT_LT(std::abs(a.q_y() - b.q_y()) , epsilon);
    EXPECT_LT(std::abs(a.q_z() - b.q_z()) , epsilon);
}

struct RobotTest : testing::Test, franka_pivot_control::FrankaPivotController {
//    std::unique_ptr<franka_pivot_control::FrankaPivotController> robot;

    RobotTest():
        FrankaPivotController(
            TEST_ROBOT_HOSTNAME, distanceEE2PP, distanceEE2PP,
            dynamicRel, cameraTilt)
    {}
};

TEST_F(RobotTest, CalcAffineFromDOFPoseInitial) {
    // start Pivoting should set the
    DOFPose initialDOFPose {0,0,0,0};
    movex::Affine initialAffine = movex::Affine(Eigen::Affine3d::Identity());
    frankx::Affine affineCalc;
    mDistanceEE2PP = 0.1;
    mCurrentAffine = initialAffine;
    mInitialPPAffine = initialAffine;
    mInitialPPAffine.translate(Eigen::Vector3d(0,0,-mDistanceEE2PP));

    calcAffineFromDOFPose(initialDOFPose, affineCalc);
    std::cout << "initialAffine:" << initialAffine.toString() << std::endl;
    std::cout << "affineCalc:"  << affineCalc.toString() << std::endl;
    EXPECT_TRUE(initialAffine.isApprox(affineCalc));
}

struct RobotCalcTest : RobotTest, testing::WithParamInterface<DOFPose> {
};

TEST_P(RobotCalcTest, calcAffineFromDOFPose_AND_calcDOFPoseFromAffine){
    double error;
    frankx::Affine testAffine;
    DOFPose resultDOFPose;
    DOFPose testDOFPose = GetParam();
    error = 0;
    calcAffineFromDOFPose(testDOFPose, testAffine);
    calcDOFPoseFromAffine(testAffine, resultDOFPose, error);
//    std::cout << "FROM:    " << testDOFPose.toString() << std::endl;
//    std::cout << "To:      " << testAffine.toString() << std::endl;
//    std::cout << "Back to: " << resultDOFPose.toString() << std::endl;
//    std::cout << "------------------------" << std::endl;
    EXPECT_LT(error, 0.0001);
    EXPECT_TRUE(testDOFPose.closeTo(resultDOFPose, 0.0001, 0.0001));
}

INSTANTIATE_TEST_SUITE_P(Default, RobotCalcTest, testing::Values(
            DOFPose({-0.3,0,0, 0}),
            DOFPose({0,0.1,0, 0}),
            DOFPose({0,0,0.2, 0}),
            DOFPose({0,0,0, 0.2}),
            DOFPose({0,0.1,0, 0.1}),
            DOFPose({-1.0,1.0,0, 0.0}),
            DOFPose({0.2,0.1,0, 0.1})
        ));

std::stringstream AffineToString(movex::Affine a)
{
    std::stringstream  ss;
    ss << " x: " << a.x() << " y: " << a.y() << " z: " << a.z()
        << " q_w: " << a.q_w() << " q_x: " << a.q_x()
        << " q_y: " << a.q_y() << " q_z: " << a.q_z();
    return ss;
}

struct RobotMoveTest : RobotTest
{
    constexpr static std::array<double, 7> goodJointPose {-0.01981415, -1.036409, -0.05556389, -2.023421, 0.01193091, 1.796796, 1.770148};
    RobotMoveTest()
    {
    }

    void testInitialPosition()
    {
        ASSERT_TRUE(mReady);
        moveJointSpace(goodJointPose);
        std::this_thread::sleep_for(5s);
        //Check we succeded moving to initial Pose
        auto it1 = goodJointPose.begin();
        auto l2 = mRobot->currentJointPositions();
        auto it2 = l2.begin();
        for(; it1 != goodJointPose.end() && it2 != l2.end(); ++it1, ++it2)
        {
            // should be smalle than some degrees
            EXPECT_LT(it1 - it2, 0.01);
        }
    }

    ~RobotMoveTest() {
        //GOOD INIITIAL POSE
        moveJointSpace(goodJointPose);
    }
};

TEST_F(RobotMoveTest, MoveToInitialPosition)
{
    testInitialPosition();
}

struct RobotCartesianTest : RobotMoveTest, testing::WithParamInterface<frankx::Affine> {
};

//TODO: test instead
TEST_P(RobotCartesianTest, MoveCartesian)
{
    testInitialPosition();
    frankx::Affine target = GetParam();
    movex::Waypoint targetWaypoint(target);
    movex::WaypointMotion targetWaypointMotion({targetWaypoint});
    mRobot->move(targetWaypointMotion);
    std::this_thread::sleep_for(5s);
    frankx::Affine current = mRobot->currentPose();
    testAffineApprox(current, target);
}

INSTANTIATE_TEST_SUITE_P(Default, RobotCartesianTest, testing::Values(
            frankx::Affine(0.1423738, -0.02001232, 0.7518987, -0.4309479, -0.3938808, -0.01671598, 0.8117033),
            frankx::Affine(0.09816521, -0.2317149, 0.7056191, -0.1678587, -0.4444543, -0.05341336, 0.8783114),
            frankx::Affine(0.09816521, -0.2417149, 0.7056191, -0.1678587, -0.4444543, -0.05341336, 0.8783114)
        ));

struct RobotJointMotionTest : RobotMoveTest, testing::WithParamInterface<std::array<double, 7>> {
};

//TODO: test instead
TEST_P(RobotJointMotionTest, MoveJointPositions)
{
    testInitialPosition();
    std::array<double, 7> target = GetParam();
    moveJointSpace(target);
    std::this_thread::sleep_for(2s);
    std::array<double, 7> current = mRobot->currentJointPositions();
    for(int i = 0; i < 7; i++)
    {
        EXPECT_LT(std::abs(target.at(i) - current.at(i)) , epsilon);
    }
}

INSTANTIATE_TEST_SUITE_P(Default, RobotJointMotionTest, testing::Values(
            std::array<double, 7>({-0.09110587, -1.281957, -0.1430118, -2.258051, -0.05919013, 1.794252, 2.516949}),
            std::array<double, 7>({-0.3715936, -1.237756, -0.05479175, -2.274433, 0.5705662, 1.583464, 1.133754}),
            std::array<double, 7>({-0.02132828, -1.337411, -0.5387088, -2.298805, 0.01366354, 1.794611, 1.597896})
        ));

int main(int argc, char *argv[])
{
    testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
