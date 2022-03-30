// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private double m_angleDelta;
  
  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(DriveSubsystem driveSubsystem, double angleDelta) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_angleDelta = angleDelta;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.setDrivePIDSetpoint(m_driveSubsystem.getAngle() + m_angleDelta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.maintainAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.resetDrivePID();
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveSubsystem.isOnTarget();
  }
}
