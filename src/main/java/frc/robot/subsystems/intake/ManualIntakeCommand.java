// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualIntakeCommand extends CommandBase {
  private final IntakeSubsystem m_intakeSubsystem;
  private final XboxController m_controller;
  private final Timer m_fullSimulationTimer = new Timer();

  /** Creates a new IntakeCommand. */
  public ManualIntakeCommand(IntakeSubsystem intakeSubsystem, XboxController controller) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  /** Create a new IntakeCommand w/o controller */
  public ManualIntakeCommand(IntakeSubsystem intakeSubsystem) {
    this(intakeSubsystem, null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intake();
    m_fullSimulationTimer.reset();
    m_fullSimulationTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller == null) return;

    if (m_fullSimulationTimer.hasElapsed(10.0)) {
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
    if (m_controller != null) {
      m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
      m_controller.setRumble(RumbleType.kRightRumble, 0.0);
    }
    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
