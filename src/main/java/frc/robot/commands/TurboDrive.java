// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurboDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private double lastSpeed;

  /** Creates a new ExampleCommand. */
  public TurboDrive(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveSubsystem = driveSubsystem;
    //addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      this.lastSpeed = this.m_driveSubsystem.speedMult;
      this.m_driveSubsystem.setDriveSpeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      this.m_driveSubsystem.setDriveSpeed(lastSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
