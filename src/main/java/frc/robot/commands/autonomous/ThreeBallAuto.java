// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

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
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    AutoTrajectory ThreeBallAuto_1 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_1", 3.0, 1.8);
    AutoTrajectory ThreeBallAuto_2 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_2", 3.0, 1.8);
    AutoTrajectory ThreeBallAuto_3 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_3", 3.0, 2.0);

    addCommands(
      // leaves tarmac, gets new ball and returns to tarmac  
      ThreeBallAuto_1.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // shoots collected ball + preloaded ball
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY, SelectedGoal.High).withTimeout(2.0),
     
      // leaves tarmac, gets new ball and returns to tarmac  
      ThreeBallAuto_2.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // shoots last ball
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY, SelectedGoal.High).withTimeout(1.5),

      // leaves tarmac
      ThreeBallAuto_3.getCommandAndStop()
    );
  }
}
