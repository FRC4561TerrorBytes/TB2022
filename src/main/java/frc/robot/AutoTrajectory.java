/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class AutoTrajectory {
  // Ramsete Command values
  final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
  //TODO: Find VOLTS_ks, VOLTS_SECONDS_PER_METER_kV, VOLTS_SECONDS_PER_METER_kA
  final double VOLTS_kS = 0.0; 
  final double VOLT_SECONDS_PER_METER_kV = 0.0;
  final double VOLT_SECONDS_SQUARED_PER_METER_kA = 0.0;
  final double kP = 0;
  final double kD = 0; 

  DriveSubsystem subsystem;
  RamseteCommand ramseteCommand;
  RamseteController disabledRamsete = new RamseteController() {
    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityMeters,
      double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
  };
  Trajectory pathplannerTrajectory;
  
  /**
   * Create new path trajectory using PathPlanner file containing path
   * @param subsystem DriveSubsystem to drive the robot
   * @param pathName PathPlanner file containing path
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AutoTrajectory(DriveSubsystem subsystem, String pathName, double maxVelocity, double maxAcceleration){
    this.subsystem = subsystem;
       
    // maxVelocity is (m/s)
    // maxAcceleration is (m/s^2)
    this.pathplannerTrajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);

    Transform2d transform = subsystem.getPose().minus(pathplannerTrajectory.getInitialPose());
    Trajectory transformedTrajectory =  pathplannerTrajectory.transformBy(transform);

    this.ramseteCommand = new RamseteCommand(
        transformedTrajectory, 
        subsystem::getPose,
        disabledRamsete,
        new SimpleMotorFeedforward(this.VOLTS_kS,
                                    this.VOLT_SECONDS_PER_METER_kV,
                                    this.VOLT_SECONDS_SQUARED_PER_METER_kA),
        this.DRIVE_KINEMATICS,
        subsystem::getWheelSpeeds,
        new PIDController(this.kP, 0, this.kD),
        new PIDController(this.kP, 0, this.kD),
        // RamseteCommand passes volts to the callback
        subsystem::tankDriveVolts,
        subsystem 
    );

  }
/**
   * Creates new path trajectory using a physical x,y coordinate points
   * @param subsystem DriveSubsystem required for drivetrain movement
   * @param waypoints list of x, y coordinate pairs in trajectory
   * @param isReversed whether the trajectory followed should be in reverse
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AutoTrajectory(DriveSubsystem subsystem, Pose2d[] waypoints, boolean isReversed, double maxVelocity, double maxAcceleration) {
    this.subsystem = subsystem;
    
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(this.VOLTS_kS,
                                    this.VOLT_SECONDS_PER_METER_kV,
                                    this.VOLT_SECONDS_SQUARED_PER_METER_kA),
        this.DRIVE_KINEMATICS,
      11);
    
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
    config.setKinematics(this.DRIVE_KINEMATICS);
    config.addConstraint(autoVoltageConstraint);
    config.setReversed(isReversed);
    
    List<Pose2d> waypointList = new ArrayList<Pose2d>();
    //waypointList.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    for(int i = 0; i < waypoints.length; i++) {
      waypointList.add(waypoints[i]);
    }

    // This transforms the starting position of the trajectory to match the starting position of the actual 
    // robot. Prevents robot from moving to first X,Y of trajectory and then following the path.
    // Changes the first point(s) of the trajectory to the X,Y point of where the robot currently is
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypointList, config);
    Transform2d transform = subsystem.getPose().minus(trajectory.getInitialPose());
    Trajectory transformedTrajectory = trajectory.transformBy(transform);

    // This is a method used to get the desired trajectory, put it into the command, have the command calculate the 
    // actual route relative to one plotted in Pathweaver, and then follow it the best it can, based on characterization given to it.
    this.ramseteCommand = new RamseteCommand(
        transformedTrajectory,  // This had been changed to be the transformed trajecotry so that it calculates trajectory 
                                // from final (transformed) trajectory
        subsystem::getPose,
        disabledRamsete,
        new SimpleMotorFeedforward(this.VOLTS_kS,
                                    this.VOLT_SECONDS_PER_METER_kV,
                                    this.VOLT_SECONDS_SQUARED_PER_METER_kA),
        this.DRIVE_KINEMATICS,
        subsystem::getWheelSpeeds,
        new PIDController(this.kP, 0, this.kD),
        new PIDController(this.kP, 0, this.kD),
        // RamseteCommand passes volts to the callback
        subsystem::tankDriveVolts,
        subsystem 
    );
  }

  /**
   * Get Ramsete command to run
   * @return Ramsete command that will stop when complete
   */
  public Command getCommand() {
    return this.ramseteCommand.andThen(() -> this.subsystem.stop());
  }
}
