// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootManualCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootDriveForwardAuto extends SequentialCommandGroup {
  /** Creates a new ShootDriveForwardAuto. */
  public ShootDriveForwardAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      // Shoots single preloaded ball
      new ShootManualCommand(shooterSubsystem, 1700.0).withTimeout(1.0),
      // Drives forward + exits tarmac
      new AutoTrajectory(driveSubsystem, "LeaveTarmac", 2.0, 1.0).getCommandAndStop()
    );
  }
}
