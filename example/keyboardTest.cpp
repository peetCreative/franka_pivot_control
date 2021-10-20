#include "FrankaPivotController.h"

#include "PivotControlMessages.h"

#include <iostream>
#include <string>
#include <vector>

bool parse_input(pivot_control_messages::DOFPose& pose, const std::string &in)
{
    std::stringstream ss(in.substr(1,in.length()));
    std::string item;
    int i = 0;
    double d;
    while (std::getline(ss, item, ',')) {
        try {
            d = std::stod(item);
        }
        catch(...) {
            return false;
        };
        switch(i)
        {
            case 0:
                pose.pitch = d;
                break;
            case 1:
                pose.yaw = d;
                break;
            case 2:
                pose.roll = d;
                break;
            case 3:
                pose.transZ = d;
                break;
        }
        i++;
    }
    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    float distanceEE2PP = 0.2;
    float dynamicRel = 0.1;
    float cameraTilt = -0.52359;
    float step = 0.1;
    //TODO: guard exceptions
    std::string ip(argv[1]);
    std::cout << "Starting Franka Panda with IP:" << ip << std::endl;
    franka_pivot_control::FrankaPivotController pivoting(
            ip, distanceEE2PP, distanceEE2PP,
            dynamicRel, cameraTilt);
    pivoting.startPivoting();
    if (!pivoting.isReady())
    {
        return 1;
    }
    std::cout << "Fot pitch: use w and s" << std::endl;
    std::cout << "Fot yaw: use a and d" << std::endl;
    std::cout << "Fot roll: use r and t" << std::endl;
    std::cout << "Fot transZ: use f and g" << std::endl;
    std::cout << "To Stop: use q" << std::endl;
    std::cout << "To Go To a specific position: use b<pitch>,<yaw>,<roll>,<trans_z>" << std::endl;
    std::cout << "To set speed: use h<speed_val> where speed_val between 0 and 1" << std::endl;
    std::cout << "Robot will move after you press any key." << std::endl;
    std::cout << "Hold the safety Button close." << std::endl;
    bool quit = false;
    pivot_control_messages::DOFPose pose;
    pivot_control_messages::DOFBoundaries boundaries;
    while(!quit)
    {
        std::string in;
        std::cout << "next: " << std::endl;
        std::cin >> in;
        if(in.length() < 1)
            return 0;
        if(!pivoting.getCurrentDOFPose(pose) ||
            !pivoting.getDOFBoundaries(boundaries))
        {
            std::cout << "An Error occured in Robot Controller" << std::endl;
            break;
        }
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
            case '0':
                //no movement
                pose.pitch = 0;
                pose.yaw = 0;
                pose.roll = 0;
                pose.transZ = 0;
                break;
            case 'b':
                parse_input(pose, in);
                break;
            case 'h':
                try {
                    double d = std::stod(in.substr(1, in.length()));
                    pivoting.stopPivoting();
                    if (!pivoting.setSpeed(d))
                        std::cout << "new speed is " << d << std::endl;
                    else
                        std::cout << "wrong speed " << d << std::endl;
                    pivoting.startPivoting();
                }
                catch (...)
                {
                    std::cout << "Wrong input for speed" << std::endl;
                    return 1;
                }
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
        std::cout << "moveTo" << pose.toString() << std::endl;
        pivoting.setTargetDOFPose(pose);
    }
}
