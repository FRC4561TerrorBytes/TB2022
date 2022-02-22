// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommandManual extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  private double m_rpm;

  /** Creates a new ShootCommand. */
  public ShootCommandManual(ShooterSubsystem shooterSubsystem, double rpm) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_rpm = rpm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFlywheelSpeed(m_rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.isFlywheelAtSpeed()) m_shooterSubsystem.feederShoot();
    else m_shooterSubsystem.feederStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.flywheelStop();
    m_shooterSubsystem.feederStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}