// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveTarmac extends SequentialCommandGroup {
  /** Creates a new LeaveTarmac. */
  public LeaveTarmac(DriveSubsystem driveSubsystem) {
    AutoTrajectory LeaveTarmac = new AutoTrajectory(driveSubsystem, "LeaveTarmac", 2.0, 1.5);

    addCommands(
      // Wait a while
      new WaitCommand(12.0),

      // Leave tarmac
      LeaveTarmac.getCommandAndStop()
    );
  }
}
