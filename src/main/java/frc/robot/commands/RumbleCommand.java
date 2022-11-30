// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * When scheduled, an instance of this command will rumble the given controller
 * until canceled. Instances should race with an instance of
 * {@link WaitCommand}. See {@link Command#raceWith(Command...)} for the best
 * way to do this.
 */
public class RumbleCommand extends CommandBase {
  private final GenericHID m_controller;

  /**
   * @param controller the controller (XBox, Gamepad, etc) to rumble.
   */
  public RumbleCommand(final GenericHID controller) {
    m_controller = controller;
  }

  /**
   * Turn the rumble on when first scheduled.
   */
  @Override
  public void initialize() {
    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
  }

  /**
   * Turn the rumble off when ending.
   */
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
  }
}
