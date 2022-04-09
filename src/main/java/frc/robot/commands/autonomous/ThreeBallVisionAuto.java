// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootVisionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallVisionAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallVisionAuto. */
  public ThreeBallVisionAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
    AutoTrajectory ThreeBallVizAuto_1 = new AutoTrajectory(driveSubsystem, "ThreeBallVizAuto_1", 3.0, 1.9);
    AutoTrajectory ThreeBallVizAuto_2 = new AutoTrajectory(driveSubsystem, "ThreeBallVizAuto_2", 3.0, 1.6);
    AutoTrajectory ThreeBallVizAuto_3 = new AutoTrajectory(driveSubsystem, "ThreeBallVizAuto_3", 3.0, 1.8);

    addCommands(
      // leaves tarmac, gets new ball and returns to tarmac  
      ThreeBallVizAuto_1.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // shoots collected ball + preloaded ball
      new ShootVisionCommand(driveSubsystem, shooterSubsystem, visionSubsystem, Constants.SHOOT_DELAY).withTimeout(2.0),
     
      // leaves tarmac, gets new ball and returns to tarmac  
      ThreeBallVizAuto_2.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // shoots last ball
      new ShootVisionCommand(driveSubsystem, shooterSubsystem, visionSubsystem, Constants.SHOOT_DELAY).withTimeout(2.0),

      // leaves tarmac
      ThreeBallVizAuto_3.getCommandAndStop()
    );
  }
}
