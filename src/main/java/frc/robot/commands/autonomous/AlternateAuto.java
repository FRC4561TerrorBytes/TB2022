// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utils.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlternateAuto extends SequentialCommandGroup {
  /** Creates a new AlternateAuto. */
  public AlternateAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    AutoTrajectory AlternateAuto_1 = new AutoTrajectory(driveSubsystem, "AlternateAuto_1", 3.0, 1.9);
    AutoTrajectory AlternateAuto_2 = new AutoTrajectory(driveSubsystem, "AlternateAuto_2", 3.0, 1.9);

    addCommands(
        // Leaves tarmac and intakes new ball and returns to hub
        new InstantCommand(intakeSubsystem::requestIntake),
        AlternateAuto_1.getCommandAndStop(),
        new InstantCommand(intakeSubsystem::requestRetraction),

        // Shoot 2 balls use to be here.

        // Leaves tarmac
        AlternateAuto_2.getCommandAndStop());
  }
}
