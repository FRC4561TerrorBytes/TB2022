// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autonomous.AlternateAuto;
import frc.robot.commands.autonomous.FourBallAutoAdvanced;
import frc.robot.commands.autonomous.LeaveTarmac;
import frc.robot.commands.autonomous.ShootDriveForwardAuto;
import frc.robot.commands.autonomous.ThreeBallAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(),
                                                                           Constants.DRIVE_kP,
                                                                           Constants.DRIVE_kD,
                                                                           Constants.DRIVE_TURN_SCALAR,
                                                                           Constants.CONTROLLER_DEADBAND,
                                                                           Constants.DRIVE_LOOKAHEAD,
                                                                           Constants.DRIVE_METERS_PER_TICK,
                                                                           Constants.DRIVE_MAX_LINEAR_SPEED,
                                                                           Constants.DRIVE_TRACTION_CONTROL_CURVE,
                                                                           Constants.DRIVE_THROTTLE_INPUT_CURVE,
                                                                           Constants.DRIVE_TURN_INPUT_CURVE,
                                                                           Constants.DRIVE_CURRENT_LIMIT_CONFIGURATION);

  private static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem(IntakeSubsystem.initializeHardware(), 
                                                                              Constants.INTAKE_ARM_CONFIG, 
                                                                              Constants.INTAKE_ROLLER_SPEED);

  private static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem(ShooterSubsystem.initializeHardware(), 
                                                                                 Constants.FLYWHEEL_MASTER_CONFIG,
                                                                                 Constants.FEEDER_INTAKE_SPEED,
                                                                                 Constants.FEEDER_SHOOT_SPEED,
                                                                                 Constants.SHOOTER_LOW_CURVE,
                                                                                 Constants.SHOOTER_HIGH_CURVE);

  private static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem(ClimberSubsystem.initializeHardware(),
                                                                                 Constants.TELESCOPE_CONFIG,
                                                                                 Constants.WINCH_CONFIG);

  private static final XboxController PRIMARY_CONTROLLER = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
  private static final XboxController SECONDARY_CONTROLLER = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

  private static SendableChooser<SequentialCommandGroup> m_automodeChooser = new SendableChooser<>();

  private static PhotonCamera m_pi = new PhotonCamera("photonvision");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings(); 

    // Set default commands for subsystems
    DRIVE_SUBSYSTEM.setDefaultCommand(
      new RunCommand(
        () -> DRIVE_SUBSYSTEM.teleopPID(PRIMARY_CONTROLLER.getLeftY(), PRIMARY_CONTROLLER.getRightX()), 
        DRIVE_SUBSYSTEM
      )
    );

    // Initialize Shuffleboard tabs
    defaultShuffleboardTab();
    DRIVE_SUBSYSTEM.shuffleboard();
    INTAKE_SUBSYSTEM.shuffleboard();
    SHOOTER_SUBSYSTEM.shuffleboard();
    CLIMBER_SUBSYSTEM.shuffleboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  @SuppressWarnings("unused")
  private void configureButtonBindings() { 
    // Primary controller buttons
    JoystickButton primaryButtonA = new JoystickButton(PRIMARY_CONTROLLER, Button.kA.value);
    JoystickButton primaryButtonB = new JoystickButton(PRIMARY_CONTROLLER, Button.kB.value);
    JoystickButton primaryButtonX = new JoystickButton(PRIMARY_CONTROLLER, Button.kX.value);
    JoystickButton primaryButtonY = new JoystickButton(PRIMARY_CONTROLLER, Button.kY.value);
    JoystickButton primaryButtonLBumper = new JoystickButton(PRIMARY_CONTROLLER, Button.kLeftBumper.value);
    JoystickButton primaryButtonRBumper = new JoystickButton(PRIMARY_CONTROLLER, Button.kRightBumper.value);
    JoystickButton primaryButtonLStick = new JoystickButton(PRIMARY_CONTROLLER, Button.kLeftStick.value);
    JoystickButton primaryButtonRStick = new JoystickButton(PRIMARY_CONTROLLER, Button.kRightStick.value);
    JoystickButton primaryButtonStart = new JoystickButton(PRIMARY_CONTROLLER, Button.kStart.value);
    JoystickButton primaryButtonBack = new JoystickButton(PRIMARY_CONTROLLER, Button.kBack.value);
    POVButton primaryDPadUp = new POVButton(PRIMARY_CONTROLLER, 0);
    POVButton primaryDPadRight = new POVButton(PRIMARY_CONTROLLER, 90);
    POVButton primaryDPadDown = new POVButton(PRIMARY_CONTROLLER, 180);
    POVButton primaryDPadLeft = new POVButton(PRIMARY_CONTROLLER, 270);
    Trigger primaryTriggerLeft = new Trigger(() -> PRIMARY_CONTROLLER.getLeftTriggerAxis() > Constants.CONTROLLER_DEADBAND);
    Trigger primaryTriggerRight = new Trigger(() -> PRIMARY_CONTROLLER.getRightTriggerAxis() > Constants.CONTROLLER_DEADBAND);

    // Primary controller button combos
    Trigger primaryTelescopeUpManual = new Trigger(() -> PRIMARY_CONTROLLER.getStartButton() && PRIMARY_CONTROLLER.getPOV() == 0);
    Trigger primaryTelescopeDownManual = new Trigger(() -> PRIMARY_CONTROLLER.getStartButton() && PRIMARY_CONTROLLER.getPOV() == 180);
    Trigger primaryWinchOutManual = new Trigger(() -> PRIMARY_CONTROLLER.getStartButton() && PRIMARY_CONTROLLER.getPOV() == 90);
    Trigger primaryWinchInManual = new Trigger(() -> PRIMARY_CONTROLLER.getStartButton() && PRIMARY_CONTROLLER.getPOV() == 270);

    // Secondary controller buttons
    JoystickButton secondaryButtonA = new JoystickButton(SECONDARY_CONTROLLER, Button.kA.value);
    JoystickButton secondaryButtonB = new JoystickButton(SECONDARY_CONTROLLER, Button.kB.value);
    JoystickButton secondaryButtonX = new JoystickButton(SECONDARY_CONTROLLER, Button.kX.value);
    JoystickButton secondaryButtonY = new JoystickButton(SECONDARY_CONTROLLER, Button.kY.value);
    JoystickButton secondaryButtonLBumper = new JoystickButton(SECONDARY_CONTROLLER, Button.kLeftBumper.value);
    JoystickButton secondaryButtonRBumper = new JoystickButton(SECONDARY_CONTROLLER, Button.kRightBumper.value);
    JoystickButton secondaryButtonLStick = new JoystickButton(SECONDARY_CONTROLLER, Button.kLeftStick.value);
    JoystickButton secondaryButtonRStick = new JoystickButton(SECONDARY_CONTROLLER, Button.kRightStick.value);
    JoystickButton secondaryButtonStart = new JoystickButton(SECONDARY_CONTROLLER, Button.kStart.value);
    JoystickButton secondaryButtonBack = new JoystickButton(SECONDARY_CONTROLLER, Button.kBack.value);
    POVButton secondaryDPadUp = new POVButton(SECONDARY_CONTROLLER, 0);
    POVButton secondaryDPadRight = new POVButton(SECONDARY_CONTROLLER, 90);
    POVButton secondaryDPadDown = new POVButton(SECONDARY_CONTROLLER, 180);
    POVButton secondaryDPadLeft = new POVButton(SECONDARY_CONTROLLER, 270);
    Trigger secondaryTriggerLeft = new Trigger(() -> SECONDARY_CONTROLLER.getLeftTriggerAxis() > Constants.CONTROLLER_DEADBAND);
    Trigger secondaryTriggerRight = new Trigger(() -> SECONDARY_CONTROLLER.getRightTriggerAxis() > Constants.CONTROLLER_DEADBAND);

    // Primary controller bindings
    primaryButtonRBumper.whenHeld(new ShootCommand(SHOOTER_SUBSYSTEM));
    primaryTriggerLeft.whileActiveOnce(new OuttakeCommand(INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM));
    primaryButtonX.whenPressed(new InstantCommand(() -> INTAKE_SUBSYSTEM.toggleArmPosition(), INTAKE_SUBSYSTEM));
    primaryButtonY.whenPressed(new InstantCommand(() -> SHOOTER_SUBSYSTEM.toggleSelectedGoal(), SHOOTER_SUBSYSTEM));
    primaryTriggerRight.whileActiveOnce(new IntakeCommand(INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM, PRIMARY_CONTROLLER));
    // primaryDPadUp.whenPressed(new InstantCommand(() -> CLIMBER_SUBSYSTEM.nextClimberState(), CLIMBER_SUBSYSTEM));
    // primaryDPadDown.whenPressed(new InstantCommand(() -> CLIMBER_SUBSYSTEM.previousClimberState(), CLIMBER_SUBSYSTEM));

    primaryDPadUp.whileHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.telescopeUpManual(), () -> CLIMBER_SUBSYSTEM.telescopeStop(), CLIMBER_SUBSYSTEM));
    primaryDPadDown.whenHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.telescopeDownManual(), () -> CLIMBER_SUBSYSTEM.telescopeStop(), CLIMBER_SUBSYSTEM));
    primaryDPadLeft.whenHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.winchOutManual(), () -> CLIMBER_SUBSYSTEM.winchStopManual(), CLIMBER_SUBSYSTEM));
    primaryDPadRight.whenHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.winchInManual(), () -> CLIMBER_SUBSYSTEM.winchStopManual(), CLIMBER_SUBSYSTEM));

    // Secondary controller bindings
    secondaryDPadUp.whileHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.telescopeUpManual(), () -> CLIMBER_SUBSYSTEM.telescopeStop(), CLIMBER_SUBSYSTEM));
    secondaryDPadDown.whenHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.telescopeDownManual(), () -> CLIMBER_SUBSYSTEM.telescopeStop(), CLIMBER_SUBSYSTEM));
    secondaryDPadLeft.whenHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.winchOutManual(), () -> CLIMBER_SUBSYSTEM.winchStopManual(), CLIMBER_SUBSYSTEM));
    secondaryDPadRight.whenHeld(new StartEndCommand(() -> CLIMBER_SUBSYSTEM.winchInManual(), () -> CLIMBER_SUBSYSTEM.winchStopManual(), CLIMBER_SUBSYSTEM));
  }

  /**
   * Initialize robot state
   */
  public void initialize() {
    INTAKE_SUBSYSTEM.initialize();
    CLIMBER_SUBSYSTEM.initialize();
  }

  /**
   * Initialize robot for teleop
   */
  public void teleopInit() {
    DRIVE_SUBSYSTEM.teleopInit();
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Leave Tarmac", new LeaveTarmac(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Shoot Drive Forward Auto", new ShootDriveForwardAuto(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM));
    m_automodeChooser.addOption("Three Ball Auto", new ThreeBallAuto(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM));
    m_automodeChooser.addOption("Four Ball Auto Advanced", new FourBallAutoAdvanced(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM));
    m_automodeChooser.addOption("Alternate Auto", new AlternateAuto(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM));
    m_automodeChooser.addOption("Do nothing", null);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_automodeChooser.getSelected();
  }

  /**
   * Configure default Shuffleboard tab
   */
  public void defaultShuffleboardTab() {
    m_pi.setDriverMode(true);
    Shuffleboard.selectTab("SmartDashboard");
    autoModeChooser();
    SmartDashboard.putData("Auto Mode", m_automodeChooser);
  }
}
