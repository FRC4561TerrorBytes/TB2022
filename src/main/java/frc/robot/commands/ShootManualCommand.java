// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootManualCommand extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  private double m_bigRPM, m_smallRPM;
  private int loops = 0;

  /** Creates a new ShootCommand. */
  public ShootManualCommand(ShooterSubsystem shooterSubsystem, double bigRPM, double smallRPM) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_bigRPM = bigRPM;
    this.m_smallRPM = smallRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFlywheelSpeed(m_bigRPM, m_smallRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.isFlywheelAtSpeed()) {
      loops++;
      if (loops > 6) m_shooterSubsystem.feederShoot();
    } else {
      loops = 0;
      m_shooterSubsystem.feederStop();
    }
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
