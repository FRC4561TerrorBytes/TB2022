// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.SelectedGoal;

public class ShootCommandVision extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private SelectedGoal m_prevSelectedGoal;
  private int m_loops = 0;
  private int m_loopNum;

  /**
   * Shoot using vision
   * @param driveSubsystem drive subsystem
   * @param shooterSubsystem shooter subsystem
   * @param visionSubsystem vision subsystem
   * @param delay shoot delay in seconds
   * @param tolerance aiming tolerance in degrees
   */
  public ShootCommandVision(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, double delay) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_visionSubsystem = visionSubsystem;
    this.m_prevSelectedGoal = m_shooterSubsystem.getSelectedGoal();
    this.m_loopNum = (int)Math.round(delay / Constants.ROBOT_LOOP_PERIOD);

    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(m_driveSubsystem);
    addRequirements(m_shooterSubsystem);
    addRequirements(m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Select high goal
    m_shooterSubsystem.selectGoal(SelectedGoal.High);

    // Disable driver mode
    m_visionSubsystem.setDriverMode(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Only run if target is valid
    if (m_visionSubsystem.isTargetValid()) {
      // Run flywheel based on distance from target
      m_shooterSubsystem.setFlywheelVision(m_visionSubsystem.getDistance());

      // Aim robot toward target
      m_driveSubsystem.aimToAngle(m_visionSubsystem.getYaw());

      // Only run feeder if flywheel is at speed and robot is on target, else stop
      if (m_shooterSubsystem.isFlywheelAtSpeed() && m_driveSubsystem.isOnTarget()) {
        m_loops++;
        if (m_loops > m_loopNum) m_shooterSubsystem.feederShoot();
      } else {
        m_loops = 0;
        m_shooterSubsystem.feederStop(false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop flywheel and feeder
    m_shooterSubsystem.flywheelStop();
    m_shooterSubsystem.feederStop(false);

    // Reset selected goal
    m_shooterSubsystem.selectGoal(m_prevSelectedGoal);

    // Re-enable driver mode
    m_visionSubsystem.setDriverMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
