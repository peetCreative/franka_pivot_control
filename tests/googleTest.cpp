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
    EXPECT_LT(std::abs(a.qW() - b.qW()) , epsilon);
    EXPECT_LT(std::abs(a.qX() - b.qX()) , epsilon);
    EXPECT_LT(std::abs(a.qY() - b.qY()) , epsilon);
    EXPECT_LT(std::abs(a.qZ() - b.qZ()) , epsilon);
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
        << " q_w: " << a.qW() << " q_x: " << a.qX()
        << " q_y: " << a.qY() << " q_z: " << a.qZ();
    return ss;
}

struct RobotMoveTest : RobotTest
{
    constexpr static std::array<double, 7> goodJointPositions {-0.01981415, -1.036409, -0.05556389, -2.023421, 0.01193091, 1.796796, 1.770148};
    RobotMoveTest()
    {
    }

    void testInitialPosition(std::array<double, 7> jointPositions = goodJointPositions)
    {
        ASSERT_TRUE(mReady);
        moveJointSpace(jointPositions);
        std::this_thread::sleep_for(5s);
        //Check we succeded moving to initial Pose
        auto it1 = jointPositions.begin();
        auto l2 = mRobot->currentJointPositions();
        auto it2 = l2.begin();
        for(; it1 != jointPositions.end() && it2 != l2.end(); ++it1, ++it2)
        {
            // should be smaller than some degrees
            EXPECT_LT(*it1 - *it2, 0.01);
        }
    }

    ~RobotMoveTest() {
        //GOOD INIITIAL POSE
        moveJointSpace(goodJointPositions);
    }
};

TEST_F(RobotMoveTest, MoveToInitialPosition)
{
    testInitialPosition();
}

struct RobotCartesianTest : RobotMoveTest, testing::WithParamInterface<double> {
};

TEST_P(RobotCartesianTest, MoveCartesian)
{
    testInitialPosition();
    frankx::Affine start = mRobot->currentPose();
    double dist = GetParam();
    frankx::Affine target = start;
    target.translate({0,0, -dist});
    moveCartesianZ(dist);
    std::this_thread::sleep_for(5s);
    frankx::Affine current = mRobot->currentPose();
    testAffineApprox(current, target);
}

INSTANTIATE_TEST_SUITE_P(Default, RobotCartesianTest,
                         testing::Values(0.1, 0.05, -0.1));

typedef std::pair<std::array<double, 7>, frankx::Affine> correspondingPoses;

struct RobotJointMotionTest : RobotMoveTest, testing::WithParamInterface<correspondingPoses> {
};

TEST_P(RobotJointMotionTest, MoveJointPositions)
{
    testInitialPosition();
    correspondingPoses target = GetParam();
    moveJointSpace(target.first);
    std::this_thread::sleep_for(2s);
    std::array<double, 7> current = mRobot->currentJointPositions();
    for(int i = 0; i < 7; i++)
    {
        EXPECT_LT(std::abs(target.first.at(i) - current.at(i)) , epsilon);
    }
    affx::Affine currentPose = mRobot->currentPose();
    affx::Affine targetPose = target.second;
    for(int i = 0; i < 16; i++)
    {
        EXPECT_LT(std::abs(targetPose.x() - currentPose.x()) , epsilon);
        EXPECT_LT(std::abs(targetPose.y() - currentPose.y()) , epsilon);
        EXPECT_LT(std::abs(targetPose.z() - currentPose.z()) , epsilon);
        EXPECT_LT(std::abs(targetPose.qW() - currentPose.qW()) , epsilon);
        EXPECT_LT(std::abs(targetPose.qX() - currentPose.qX()) , epsilon);
        EXPECT_LT(std::abs(targetPose.qY() - currentPose.qY()) , epsilon);
        EXPECT_LT(std::abs(targetPose.qZ() - currentPose.qZ()) , epsilon);
    }

}

INSTANTIATE_TEST_SUITE_P(Default, RobotJointMotionTest, testing::Values(
    correspondingPoses({std::array<double, 7>({-0.09110587, -1.281957, -0.1430118, -2.258051, -0.05919013, 1.794252, 2.516949}),
        frankx::Affine(0.1219556, -0.08218796, 0.7718618, -0.19257, -0.380102, 0.114439, 0.897409)}),
    correspondingPoses({std::array<double, 7>({-0.3715936, -1.237756, -0.05479175, -2.274433, 0.5705662, 1.583464, 1.133754}),
        frankx::Affine(0.1421763, -0.02031988, 0.7521502, 0.7084383, 0.370412, -0.1356935, -0.5852327)}),
    correspondingPoses({std::array<double, 7>({-0.02132828, -1.337411, -0.5387088, -2.298805, 0.01366354, 1.794611, 1.597896}),
        frankx::Affine(0.09802533, -0.2318589, 0.7058432, -0.4908422, -0.4311694, 0.1211248, 0.7473257)})
));

struct PivotTargetPose
{
    DOFPose pose;
    int waitThere;
    bool expectReaching;
};

struct TargetTrajectory
{
    std::string name;
    std::vector<PivotTargetPose> poses;
    double speed;
    //! check that franka move smoothly with this speed
    bool check_for_franka_errors;

    friend std::ostream& operator<<(std::ostream& o, TargetTrajectory const& a) {
        o << a.name;
        return o;
    }
};

struct RobotPivotMotionTest : RobotMoveTest, testing::WithParamInterface<TargetTrajectory> {
};

TEST_P(RobotPivotMotionTest, TestPivoting)
{
    // move to good ausgang position
    testInitialPosition({-0.03665655, -1.02152, 0.000872533, -2.293926, 0.01398768, 1.902754, 0.785967});
    setSpeed(GetParam().speed);
    while (!mRobot->recoverFromErrors()){}
    startPivoting();

    // for loop to move around meassuring the time needed to move to this position
    for(auto target: GetParam().poses)
    {
        bool succ_set = setTargetDOFPose(target.pose);
        EXPECT_TRUE(succ_set);
        std::cout << "move" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(target.waitThere));
        DOFPose pose;
        bool succ_get = getCurrentDOFPose(pose);
        EXPECT_TRUE(succ_get);
        if(target.expectReaching)
        {
            std::cout << "target : " << target.pose.toString() << std::endl;
            std::cout << "current: " << pose.toString() << std::endl;
            EXPECT_TRUE(target.pose.closeTo(pose, 0.1, epsilon));
        }
    }
    stopPivoting();
    if(GetParam().check_for_franka_errors)
    {
        std::string franka_error;
        EXPECT_FALSE(getFrankaError(franka_error));
    }
    //TODO: check that the first joint does not move to far
}

INSTANTIATE_TEST_SUITE_P(Default, RobotPivotMotionTest, testing::Values(
        TargetTrajectory({"Slow and with gaps",
            {
                PivotTargetPose({{0,0,0,0}, 1000, true}),
                PivotTargetPose({{0,0,0.3,0}, 2000, true}),
                PivotTargetPose({{0,0.1,0.5,0}, 2000, true}),
                PivotTargetPose({{0.2,0.01,0.02,0}, 2000, true})
            }, 0.2, true}),
        TargetTrajectory({"Slow and without gaps",
            {
                PivotTargetPose({{0,0,0,0}, 10, true}),
                PivotTargetPose({{0,0,0.3,0}, 100, false}),
                PivotTargetPose({{0,0.1,0.2,0.02}, 100, false}),
                PivotTargetPose({{0,0.1,0.5,0}, 2000, true}),
                PivotTargetPose({{0.2,0.01,0.02,0}, 2000, true})
            }, 0.2, true}),
        TargetTrajectory({"Fast and with gaps",
            {
                PivotTargetPose({{0,0,0,0}, 1000, true}),
                PivotTargetPose({{0,0,0.3,0}, 2000, true}),
                PivotTargetPose({{0,0.1,0.5,0}, 2000, true}),
                PivotTargetPose({{0.2,0.01,0.02,0}, 2000, true})
            }, 0.4, false}),
        TargetTrajectory({"Fast and without gaps",
            {
                PivotTargetPose({{0,0,0,0}, 10, true}),
                PivotTargetPose({{0,0,0.3,0}, 100, false}),
                PivotTargetPose({{0,0.1,0.2,0.02}, 100, false}),
                PivotTargetPose({{0,0.1,0.5,0}, 2000, true}),
                PivotTargetPose({{0.2,0.01,0.02,0}, 2000, true})
            }, 0.4, false})
        ));

int main(int argc, char *argv[])
{
    testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
