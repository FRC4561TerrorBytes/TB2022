// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FlywheelSpeed;

public class SpitOutCommand extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  private double m_bigRPM, m_smallRPM;
  private int m_loops = 0;
  private int m_loopNum;

  /** Creates a new ShootCommand. */
  public SpitOutCommand(ShooterSubsystem shooterSubsystem, double delay, FlywheelSpeed flywheelSpeed) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_bigRPM = flywheelSpeed.getBigFlywheelSpeed();
    this.m_smallRPM = flywheelSpeed.getSmallFlywheelSpeed();

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
      m_loops++;
      if (m_loops > m_loopNum) m_shooterSubsystem.feederShoot();
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
