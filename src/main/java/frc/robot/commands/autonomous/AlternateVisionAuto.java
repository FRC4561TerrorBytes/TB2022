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
public class AlternateVisionAuto extends SequentialCommandGroup {
  /** Creates a new AlternateVisionAuto. */
  public AlternateVisionAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
    AutoTrajectory AlternateVizAuto_1 = new AutoTrajectory(driveSubsystem, "AlternateVizAuto_1", 3.0, 1.9);
    AutoTrajectory AlternateVizAuto_2 = new AutoTrajectory(driveSubsystem, "AlternateVizAuto_2", 3.0, 1.9);
    
    addCommands(
      // Leaves tarmac and intakes new ball and returns to hub
      AlternateVizAuto_1.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),
      
      // Shoots balls
      new ShootVisionCommand(driveSubsystem, shooterSubsystem, visionSubsystem, Constants.SHOOT_DELAY).withTimeout(1.5),

      // Leave tarmac
      AlternateVizAuto_2.getCommandAndStop()
    );
  }
}
