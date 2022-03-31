// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.SelectedGoal;
import frc.robot.utils.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAutoAdvanced extends SequentialCommandGroup {
  /** Creates a new FourBallAutoAdvanced. */
  public FourBallAutoAdvanced(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    AutoTrajectory FourBallAuto_1 = new AutoTrajectory(driveSubsystem, "FourBallAuto_1", 3.0, 3.0);
    AutoTrajectory FourBallAuto_2 = new AutoTrajectory(driveSubsystem, "FourBallAuto_2", 3.5, 3.0);
    AutoTrajectory FourBallAuto_3 = new AutoTrajectory(driveSubsystem, "FourBallAuto_3", 3.0, 2.0);

    addCommands(
      // Leaves tarmac, gets new ball and returns to tarmac  
      FourBallAuto_1.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // Shoots collected + preloaded ball
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY, SelectedGoal.High).withTimeout(1.0),
      
      // Leaves tarmac, gets 2 new balls and returns to tarmac
      FourBallAuto_2.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // Shoots balls
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY, SelectedGoal.High).withTimeout(1.0),
      
      // Leaves tarmac
      FourBallAuto_3.getCommandAndStop()
    );
  }
}
