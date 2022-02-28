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

### [Intake Subsystem](src/main/java/frc/robot/subsystems/IntakeSubsystem.java)
The intake subsystem controls the cargo collection system of the robot. It uses 1 Falcon 500 motor to power the motion of the arm, and 1 NEO 550 for the intake roller. The intake system uses a four bar linkage to extend arms and pull in cargo. We also used PID to correct the arm position, motion profiles to control motor acceleration during PID correction.
When intake is called the arm is brought down, ensuring the ball collection is on the ground.
We used the Falcon 500 motors for their power and for their built in enoders, allowing for PID and motion profiling. 

Key points:
* Four bar linkage intake mechanism
* Powered by 1 Falcon 500 and 1 NEO 550
* Motion Profiling for smooth arm operation

### [Shooter Subsystem](src\main\java\frc\robot\subsystems\ShooterSubsystem.java)
The shooter subsystem controls the shooting mechanism of the robot. It uses 2 Falcon 500s for the flywheel and 2 NEOs for the feeder wheels. We use PID for the flywheel to ensure that the balls are being shot when the flywheel is at the correct speed. The feeder works with our intake to bring it balls and the feeder wheels will stop automatically when a ball is detected. Balls are detected using an infrared beam break sensor. The upper feeder wheel stops when the upper sensor is triggered, and the lower feeder wheel stops when both sensors are triggered. 

Key points:
* Powered by 2 Falcon 500s and 2 NEOs
* 2 beam break sensors 
* Automated feeder logic using OR gate
* Motion profiling for smooth arm operation

### [Climber Subsystem](src/main/java/frc/robot/subsystems/ClimberSubsystem.java)
The climber subsystem controls the 3 dimensional movement of the robot both on the y and z planes. It uses 3 Falcon 500 motors total, 2 to power the telescoping arms that reach out to the next set of rungs in order to "traverse" bar to bar, and another on the winch, the mechanism that lets the robot lean the telescoping arms at an angle to be able to reach the next rung, which is diagonal from the current rung. The climber subsystem makes use of 2 sets of arms and hooks, one telescoping, and one fixed. The fixed arm acts as an anchor to hold the robot on a rung, while the telescoping arm reaches out and latches onto the next rung. The TalonFXs are used to find the relative position of the telescoping arm. 
A state machine and iterator is used to make a climber sequence, which checks for driver input to continue from state to state or in order to go back a step in case a step is completed incorrectly. The state machine also checks to make sure that a step has been completed correctly, by checking if the number of ticks that the motors have rotated matches the correct output.

Key points: 
* Powered by 3 Falcon 500s
* 2 Arms - Telescoping and Fixed w/ Hooks
* Winch for changing telescoping arm reach angle
* State machine and iterator for simple climber sequence w/ input from 2 driver buttons
