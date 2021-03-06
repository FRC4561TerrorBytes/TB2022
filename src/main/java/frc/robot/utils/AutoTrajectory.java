/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTrajectory {
  // Ramsete Command values
  private final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
  private final double VOLTS_kS = 0.74849; 
  private final double VOLT_SECONDS_PER_METER_kV = 2.4385;
  private final double VOLT_SECONDS_SQUARED_PER_METER_kA = 0.29283;
  private final double kP = 1;
  private final double kD = 0; 
  private final double kRamseteB = 2.0;
  private final double kRamseteZeta = 0.7;
  private final double MAX_VOLTAGE = 11.0;

  DriveSubsystem m_driveSubsystem;
  RamseteCommand m_ramseteCommand;
  Trajectory m_pathplannerTrajectory;
  
  /**
   * Create new path trajectory using PathPlanner path
   * @param driveSubsystem DriveSubsystem to drive the robot
   * @param pathName PathPlanner path name
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, String pathName, double maxVelocity, double maxAcceleration) {
    this.m_driveSubsystem = driveSubsystem;

    m_pathplannerTrajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);

    RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    ramseteController.setEnabled(true);

    m_ramseteCommand = new RamseteCommand(
        m_pathplannerTrajectory, 
        m_driveSubsystem::getPose,
        ramseteController,
        new SimpleMotorFeedforward(VOLTS_kS,
                                   VOLT_SECONDS_PER_METER_kV,
                                   VOLT_SECONDS_SQUARED_PER_METER_kA),
        DRIVE_KINEMATICS,
        m_driveSubsystem::getWheelSpeeds,
        new PIDController(kP, 0, kD),
        new PIDController(kP, 0, kD),
        // RamseteCommand passes volts to the callback
        m_driveSubsystem::autoTankDriveVolts,
        m_driveSubsystem 
    );
  }

  /**
   * Creates new path trajectory using a physical x,y coordinate points
   * @param driveSubsystem DriveSubsystem required for drivetrain movement
   * @param waypoints list of x, y coordinate pairs in trajectory
   * @param isReversed whether the trajectory followed should be in reverse
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, Pose2d[] waypoints, boolean isReversed, double maxVelocity, double maxAcceleration) {
    this.m_driveSubsystem = driveSubsystem;
    
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(VOLTS_kS,
                                   VOLT_SECONDS_PER_METER_kV,
                                   VOLT_SECONDS_SQUARED_PER_METER_kA),
        DRIVE_KINEMATICS,
        MAX_VOLTAGE
      );
    
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
    config.setKinematics(DRIVE_KINEMATICS);
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
    Transform2d transform = driveSubsystem.getPose().minus(trajectory.getInitialPose());
    Trajectory transformedTrajectory = trajectory.transformBy(transform);

    RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    ramseteController.setEnabled(true);

    // This is a method used to get the desired trajectory, put it into the command, have the command calculate the 
    // actual route relative to one plotted in Pathweaver, and then follow it the best it can, based on characterization given to it.
    m_ramseteCommand = new RamseteCommand(
        transformedTrajectory,
        driveSubsystem::getPose,
        ramseteController,
        new SimpleMotorFeedforward(VOLTS_kS,
                                   VOLT_SECONDS_PER_METER_kV,
                                   VOLT_SECONDS_SQUARED_PER_METER_kA),
        DRIVE_KINEMATICS,
        m_driveSubsystem::getWheelSpeeds,
        new PIDController(kP, 0, kD),
        new PIDController(kP, 0, kD),
        // RamseteCommand passes volts to the callback
        m_driveSubsystem::autoTankDriveVolts,
        m_driveSubsystem 
    );
  }

  /**
   * Reset drive odometry to beginning of this path
   */
  public void resetOdometry() {
    m_driveSubsystem.resetOdometry(m_pathplannerTrajectory.getInitialPose());
  }

  /**
   * Get Ramsete command to run
   * @return Ramsete command that will stop when complete
   */
  public Command getCommandAndStop() {
    return new InstantCommand(() -> resetOdometry(), m_driveSubsystem).andThen(
      m_ramseteCommand.withTimeout(m_pathplannerTrajectory.getTotalTimeSeconds()).andThen(() -> {
        m_driveSubsystem.stop();
      })
    );
  }
  
  /**
   * Get Ramsete command to run
   * @return Ramsete command that does NOT stop when complete
   */
  public Command getCommand() {
    return new InstantCommand(() -> resetOdometry(), m_driveSubsystem).andThen(
      m_ramseteCommand.withTimeout(m_pathplannerTrajectory.getTotalTimeSeconds())
    );
  }
}
