// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootManualCommand extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  private double m_bigRPM, m_smallRPM;
  private int m_loops = 0;
  private int m_loopNum;

  /** Creates a new ShootCommand. */
  public ShootManualCommand(ShooterSubsystem shooterSubsystem, double delay, double bigRPM, double smallRPM) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_bigRPM = bigRPM;
    this.m_smallRPM = smallRPM;

    m_loopNum = (int)Math.round(delay / Constants.ROBOT_LOOP_PERIOD);

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
      m_loops++;
      if (m_loops > m_loopNum) m_shooterSubsystem.feederShoot();
    } else {
      m_loops = 0;
      m_shooterSubsystem.feederStop(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.flywheelStop();
    m_shooterSubsystem.feederStop(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
