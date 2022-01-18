# FRC 2022
Team 4561's 2022 robot code. Robot's code is written in Java and is based off of WPILib's Java control system.

The code is organised into several subsystems, each responsible for a different aspect of the robot's functionality. This document explains this code.

## Subsystems
### Drive Subsystem
Subsystem responsible for driving the robot.

    *Differential Drive - Gearboxes on both sides of drivetrain

    *PID Loops in DriveSubsystem - Look for error and corrects for it using feedback from the gyro, helps us drive without drift or offset

    *Odometry - Uses navX gyro and encoders to make sure the robot is facing and traveling in the right direction

    *Traction Control - Optimizes handling of drivetrain, limits wheel slip, reduces tipping


### Automodes
Controls robot movement and functions during autonomous period.

    *Odometry - Uses odometry to estimate current position of robot, uses encoders to measure distance and find position

    *PathPlanner - Tool used to plot routes we want the robot to follow autonomously. Generates a .path file containing (x,y) points
