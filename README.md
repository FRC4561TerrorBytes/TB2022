# FRC 2022
Team 4561's 2022 robot code. Robot's code is written in Java and is based off of WPILib's Java control system.

The code is organised into several subsystems, each responsible for a different aspect of the robot's functionality. This document explains this code.

## [Subsystems](src/main/java/frc/robot/subsystems)
### [Drive Subsystem](src/main/java/frc/robot/subsystems/DriveSubsystem.java)
The drive subsystem controls the drivetrain of the robot, which is a 6-wheel differential drive. Each side of the robot is driven by a set of 2 Falcon 500 brushless motors, providing ample power for our robot.
The Falcon 500 also provides the benefit of having a built-in encoder to accuately measure wheel rotations. The motors are connected to the wheels by a gearbox with a 7.64:1 gear ratio.
We also implement a closed loop control system for our drivetrain, using the yaw axis gyroscope on the navX IMU for feedback. This allows the driver to drive perfectly straight, as well as allowing the robot to maintain orientation when being hit by other robots, driving over rough terrain, etc. This closed loop control also includes a rudimentary traction control system, allowing the robot to limit wheel slip, and help prevent unintentional tipping in the pitch axis. 
Lastly, the drivetrain also implements odometry, which allows the robot to use the IMU and encoder data to accuately determine its position on the field, a useful feature for the autonomous period.

Key points:
* 6 wheel differential drive
* Powered by 4 Falcon 500 brushless motors
* Closed loop drive control system with traction control
* Odometry for accurate positioning during autonomous
