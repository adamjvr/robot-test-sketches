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

#include <ArduinoEigen.h>
#include "robot_manipulator.h"

void printMatrix(const Eigen::Matrix3d& mat) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Serial.print(mat(i, j));
      Serial.print("\t");
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(9600);

  // Define joint parameters for a simple 3-DOF robot
  JointParameters joints[] = {
    {1.0, 0.0, 0.0, 0.0},  // Joint 1
    {1.0, 0.0, 0.0, 0.0},  // Joint 2
    {1.0, 0.0, 0.0, 0.0}   // Joint 3
  };

  // Create a RobotManipulator instance
  RobotManipulator robot(joints, 3);

  // Define joint angles for a specific robot configuration
  double jointAngles[] = {0.1, 0.2, 0.3};

  // Forward Kinematics
  ForwardKinematicsResult fkResult = robot.forwardKinematics(jointAngles);

  // Print Forward Kinematics results
  Serial.println("Forward Kinematics Result:");
  Serial.print("End-Effector Position: ");
  Serial.print(fkResult.position[0]);
  Serial.print(", ");
  Serial.print(fkResult.position[1]);
  Serial.print(", ");
  Serial.println(fkResult.position[2]);
  Serial.println("End-Effector Orientation:");
  printMatrix(fkResult.orientation);

  // Inverse Kinematics target position and orientation
  Eigen::Vector3d targetPosition(1.5, 0.5, 1.0);
  Eigen::Matrix3d targetOrientation;
  targetOrientation << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;

  // Inverse Kinematics
  InverseKinematicsResult ikResult = robot.inverseKinematics(targetPosition, targetOrientation);

  // Print Inverse Kinematics results
  Serial.println("\nInverse Kinematics Result:");
  if (ikResult.success) {
    Serial.print("Joint Angles: ");
    for (int i = 0; i < 3; ++i) {
      Serial.print(ikResult.jointAngles[i]);
      Serial.print(", ");
    }
    Serial.println();
  } else {
    Serial.println("Inverse Kinematics failed to converge.");
  }
}

void loop() {
  // Nothing to do in the loop for this example
}
