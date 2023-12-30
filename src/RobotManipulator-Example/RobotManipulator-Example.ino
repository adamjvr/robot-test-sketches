/*
MIT License

Copyright (c) 2023 Adam Vadala-Roth

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "robot_manipulator.h"

// Define joint parameters for a 3-DOF robot
JointParameters robotJoints[] = {
    {1.0, 0.0, 0.0, 0.0},
    {2.0, 0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 0.0}
};

// Instantiate RobotManipulator for the robot
RobotManipulator robot(robotJoints, 3);

void setup() {
    Serial.begin(9600);
    delay(1000);

    // Example: Forward Kinematics
    double jointAngles[] = {0.1, 0.2, 0.3};
    ForwardKinematicsResult fkResult = robot.forwardKinematics(jointAngles);
    
    // Print results
    Serial.println("Forward Kinematics Result:");
    Serial.print("Position: ");
    Serial.print(fkResult.position.transpose());
    Serial.println();
    Serial.println("Orientation:");
    Serial.print(fkResult.orientation);
    Serial.println();
    
    // Example: Inverse Kinematics
    Eigen::Vector3d targetPosition(2.0, 1.0, 0.5);
    Eigen::Matrix3d targetOrientation = Eigen::Matrix3d::Identity();
    InverseKinematicsResult ikResult = robot.inverseKinematics(targetPosition, targetOrientation);

    // Print results
    Serial.println("Inverse Kinematics Result:");
    if (ikResult.success) {
        Serial.print("Computed Joint Angles: ");
        for (int i = 0; i < 3; ++i) {
            Serial.print(ikResult.jointAngles[i], 4);
            if (i < 2) Serial.print(", ");
        }
        Serial.println();
    } else {
        Serial.println("Inverse Kinematics did not converge.");
    }
}

void loop() {
    // Nothing to do in the loop
}
