// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * If added to WPILIB, SequentialCommandGroup could be replaced with this.
 */
public class SequentialCommandGroup2 extends RandomAccessCommandGroup {
  /**
   * Creates a new SequentialCommandGroup2. The given commands will be run
   * sequentially, with the CommandGroup finishing when the last command finishes.
   *
   * @param commands the commands to include in this group.
   */
  public SequentialCommandGroup2(final Command... commands) {
    super(commands);
  }
}
