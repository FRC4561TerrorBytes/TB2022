package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.SelectedGoal;
import frc.robot.utils.AutoTrajectory;

public class FiveBallAuto extends ThreeBallAuto {

  public FiveBallAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    super(driveSubsystem, intakeSubsystem, shooterSubsystem);
    AutoTrajectory FiveBallAuto_1 = new AutoTrajectory(driveSubsystem, "FiveBallAuto_1", 3.0, 1.5);
    AutoTrajectory FiveBallAuto_2 = new AutoTrajectory(driveSubsystem, "FiveBallAuto_2", 3.0, 1.5);

    addCommands(
      // Get two balls from the terminal
      FiveBallAuto_1.getCommandAndStop().deadlineWith(new IntakeCommand(intakeSubsystem, shooterSubsystem)),

      // Shoot balls
      new ShootCommand(shooterSubsystem, Constants.SHOOT_DELAY, SelectedGoal.Low),

      // Leave tarmac
      FiveBallAuto_2.getCommandAndStop(),

      // Reset drive PID and reverse motors again
      new InstantCommand(() -> driveSubsystem.teleopInit(), driveSubsystem)
    );
  }
}
