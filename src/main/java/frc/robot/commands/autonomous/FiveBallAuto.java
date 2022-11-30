package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.ManualIntakeCommand;
import frc.robot.utils.AutoTrajectory;

public class FiveBallAuto extends SequentialCommandGroup {

  public FiveBallAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    AutoTrajectory FiveBallAuto_1 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_1", 3.0, 1.5);
    AutoTrajectory FiveBallAuto_2 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_2", 3.0, 1.5);
    AutoTrajectory FiveBallAuto_3 = new AutoTrajectory(driveSubsystem, "FiveBallAuto_3", 3.0, 1.5);
    AutoTrajectory FiveBallAuto_4 = new AutoTrajectory(driveSubsystem, "ThreeBallAuto_3", 3.0, 1.5);
    
    addCommands(
      // Leaves tarmac, gets a ball, and returns
      FiveBallAuto_1.getCommandAndStop().deadlineWith(new ManualIntakeCommand(intakeSubsystem)),

      // Leave tarmac again and get one more ball, then return
      FiveBallAuto_2.getCommandAndStop().deadlineWith(new ManualIntakeCommand(intakeSubsystem)),

      // Get two balls from the terminal
      FiveBallAuto_3.getCommandAndStop().deadlineWith(new ManualIntakeCommand(intakeSubsystem)),

      // Leave tarmac
      FiveBallAuto_4.getCommandAndStop()
    );
  }
}
