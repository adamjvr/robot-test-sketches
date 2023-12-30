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
