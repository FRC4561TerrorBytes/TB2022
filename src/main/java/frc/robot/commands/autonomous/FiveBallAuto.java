package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AutoTrajectory;

public class FiveBallAuto extends SequentialCommandGroup {

  public FiveBallAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    AutoTrajectory FiveBallAuto_1 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_1", 3.0, 1.5);
    AutoTrajectory FiveBallAuto_2 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_2", 3.0, 1.5);
    AutoTrajectory FiveBallAuto_3 = new AutoTrajectory(driveSubsystem, "FiveBallAuto_3", 3.0, 1.5);
    AutoTrajectory FiveBallAuto_4 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_3", 3.0, 1.5);
    
    addCommands(
      // Toggle to high goal
      new InstantCommand(() -> shooterSubsystem.toggleSelectedGoal(), shooterSubsystem),

      // Leaves tarmac, gets a ball, and returns
      FiveBallAuto_1.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),

      // Shoot both stored ball and new ball
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY),

      // Leave tarmac again and get one more ball, then return
      FiveBallAuto_2.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),

      // Shoot ball
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY),

      // Get two balls from the terminal
      FiveBallAuto_3.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),

      // Shoot balls
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY),

      // Leave tarmac
      FiveBallAuto_4.getCommandAndStop(),

      // Reset drive PID and reverse motors again
      new InstantCommand(() -> driveSubsystem.teleopInit(), driveSubsystem)
    );
  }
}
