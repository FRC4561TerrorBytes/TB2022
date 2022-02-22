// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      // leaves tarmac, gets new ball and returns to tarmac  
      new AutoTrajectory(driveSubsystem, "ThreeBallAuto_1", 1.8, 1.8).getCommandAndStop().alongWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // shoots 1 collected + 1 preloaded balls
      new ShootCommand(shooterSubsystem, 1000.0).withTimeout(3),
     
      // leaves tarmac, gets new ball and returns to tarmac  
      new AutoTrajectory(driveSubsystem, "ThreeBallAuto_2", 1.8, 1.8).getCommandAndStop().alongWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // shoots last ball
      new ShootCommand(shooterSubsystem, 1000.0).withTimeout(3),

      // leaves tarmac
      new AutoTrajectory(driveSubsystem, "ThreeBallAuto_3", 1.8, 1.5).getCommandAndStop()
    );
  }
}