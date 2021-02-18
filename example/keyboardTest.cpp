#include "FrankaPivotControl.h"
#include <iostream>


int main(int argc, char *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    franka_pivot_control::DOFBoundaries boundaries =
            {
            0.5, -0.5,
            1.2,-1.2,
            1.2, -1.2,
            0.32, 0
            };
    float distanceEE2PP = 0.2;
    float maxWaypointDist = 0.01;
    float cameraTilt = -0.52359;
    float step = 0.05;
    //TODO: guard exceptions
    franka_pivot_control::FrankaPivotControl pivoting(
            argv[1], boundaries,
            distanceEE2PP, maxWaypointDist, cameraTilt);
    std::cout << "Robot will move after you press any key." << std::endl;
    std::cout << "Hold the safety Button close." << std::endl;
    std::cout << "Fot pitch: use w and s" << std::endl;
    std::cout << "Fot yaw: use a and d" << std::endl;
    std::cout << "Fot roll: use r and t" << std::endl;
    std::cout << "Fot transZ: use f and g" << std::endl;
    std::cout << "To Stop: use q" << std::endl;
    int start;
    std::cin >> start;
    if (start == 0)
        return 0;
    bool quit = false;
    franka_pivot_control::DOFPose pose;
    while(!quit)
    {
        std::string in;
        std::cout << "next: " << std::endl;
        std::cin >> in;
        if(in.length() < 1)
            return 0;
        pose = pivoting.getCurrentDOFPose();
        char inChar = in.at(0);
        switch (inChar) {
            case 'w':
                pose.pitch += step;
                break;
            case 's':
                pose.pitch -= step;
                break;
            case 'a':
                pose.yaw += step;
                break;
            case 'd':
                pose.yaw -= step;
                break;
            case 'r':
                pose.roll += step;
                break;
            case 't':
                pose.roll -= step;
                break;
            case 'f':
                pose.transZ += step;
                break;
            case 'g':
                pose.transZ -= step;
                break;
            case 'q':
                return 0;
                break;
        }
        pose.yaw = std::min(pose.yaw, boundaries.yawMax);
        pose.yaw = std::max(pose.yaw, boundaries.yawMin);
        pose.pitch = std::min(pose.pitch, boundaries.pitchMax);
        pose.pitch = std::max(pose.pitch, boundaries.pitchMin);
        pose.roll = std::min(pose.roll, boundaries.rollMax);
        pose.roll = std::max(pose.roll, boundaries.rollMin);
        pose.transZ = std::min(pose.transZ, boundaries.transZMax);
        pose.transZ = std::max(pose.transZ, boundaries.transZMin);
        pivoting.setTargetDOFPose(pose);
    }
}
