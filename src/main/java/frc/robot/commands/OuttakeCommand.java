// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OuttakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsytem;
  private ShooterSubsystem m_shooterSubsystem;
  /** Creates a new OuttakeCommand. */
  public OuttakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.m_intakeSubsytem = intakeSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsytem);
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsytem.outtake();
    m_shooterSubsystem.feederOuttake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsytem.stop();
    m_shooterSubsystem.feederStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
