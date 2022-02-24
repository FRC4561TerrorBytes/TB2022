// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private XboxController m_controller;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, XboxController controller) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
    addRequirements(m_shooterSubsystem);
  }

  /** Create a new IntakeCommand w/o controller */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this(intakeSubsystem, shooterSubsystem, null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intake();
    m_shooterSubsystem.feederIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller == null) return;

    if (m_shooterSubsystem.isFeederFull()) {
      m_controller.setRumble(RumbleType.kLeftRumble, 1.0);
      m_controller.setRumble(RumbleType.kRightRumble, 1.0);
    } else {
      m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
      m_controller.setRumble(RumbleType.kRightRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
    m_shooterSubsystem.feederStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
