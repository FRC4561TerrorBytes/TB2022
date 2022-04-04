// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FlywheelSpeed;

public class ShootManualCommand extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  private Supplier<FlywheelSpeed> m_flywheelSpeedSupplier;
  private int m_loops = 0;
  private int m_loopNum;

  /** Creates a new ShootCommand. */
  public ShootManualCommand(ShooterSubsystem shooterSubsystem, double delay, Supplier<FlywheelSpeed> flywheelSpeedSupplier) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_flywheelSpeedSupplier = flywheelSpeedSupplier;

    m_loopNum = (int)Math.round(delay / Constants.ROBOT_LOOP_PERIOD);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFlywheelSpeed(m_flywheelSpeedSupplier.get().getBigFlywheelSpeed(),
                                        m_flywheelSpeedSupplier.get().getSmallFlywheelSpeed());
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
