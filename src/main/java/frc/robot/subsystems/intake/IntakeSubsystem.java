// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utils.TalonPIDConfig;


public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  static class Hardware {
    private WPI_TalonFX armMotor;
    private CANSparkMax rollerMotor;

    Hardware(WPI_TalonFX armMotor, 
                    CANSparkMax rollerMotor) {
      this.armMotor = armMotor;
      this.rollerMotor = rollerMotor;
    }
  }

  /** An enumeration of valid intake arm positions. */
  private enum ArmPosition {
    /** The starting and stopped, intake retracted position. */
    Top(Constants.INTAKE_ARM_CONFIG.getLowerLimit()),
    /** The intake extended for intake or outtake operation. */
    Bottom(Constants.INTAKE_ARM_CONFIG.getUpperLimit());

    /** The intake arm positional PID control set point. */
    public final double setPoint;

    /**
     * Creates a new arm position.
     * 
     * @param setPoint the intake arm positional PID set point.
     */
    private ArmPosition(double setPoint) {
      this.setPoint = setPoint;
    }
  }

  private final String SUBSYSTEM_NAME = "Intake Subsystem";

  private WPI_TalonFX m_armMotor;
  private CANSparkMax m_rollerMotor;
  private TalonPIDConfig m_armConfig;
  /** Holds the last set {@link ArmPosition} */
  private ArmPosition m_armPosition;
  /** Holds the fixed roller speed (TODO make this 2x drive speed) */
  private double m_rollerSpeed;
  /** Used to control the intake of cargo (see {@link IntakeRequest}). */
  private final AtomicReference<IntakeRequest> m_request = new AtomicReference<>(IntakeRequest.NO_REQUEST_PENDING);
  private final IntakeStateMachine m_stateMachine;

  /**
   * This enum describes the steps in handling the extension and retraction of the
   * intake.
   */
  private enum IntakeRequest {
    /** No current state change request. */
    NO_REQUEST_PENDING,
    /** Extension and intake requested. */
    INTAKE_REQUESTED,
    /** Extension and outtake requested. */
    OUTTAKE_REQUESTED,
    /** Retraction and stop of the intake has been requested. */
    RETRACTION_REQUESTED;
  }

  /**
   * Create an instance of IntakeSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param armConfig PID config for arm
   * @param rollerSpeed Intake roller speed [-1.0, +1.0]
   * @param rumbleController the driver controller to rumble as needed.
   */
  public IntakeSubsystem(final Hardware intakeHardware, final TalonPIDConfig armConfig, final double rollerSpeed, final GenericHID rumbleController) {
    this.m_armMotor = intakeHardware.armMotor;
    this.m_rollerMotor = intakeHardware.rollerMotor;
    this.m_armConfig = armConfig;
    this.m_rollerSpeed = rollerSpeed;

    m_armPosition = ArmPosition.Top;
    
    m_rollerMotor.setIdleMode(IdleMode.kCoast);
    m_rollerMotor.setInverted(true);

    m_armConfig.initializeTalonPID(m_armMotor, FeedbackDevice.IntegratedSensor);
    m_armMotor.setNeutralMode(NeutralMode.Brake);
    m_armMotor.setSelectedSensorPosition(0.0);
    m_stateMachine = new IntakeStateMachine(this, rumbleController);
    setDefaultCommand(this.m_stateMachine.getDefaultCommand());
        new Trigger(this::isStateMachineRunning)
                .whenInactive(new InstantCommand(() -> m_request.set(IntakeRequest.NO_REQUEST_PENDING)));
  }

  /**
   * Initialize hardware devices for intake subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(new WPI_TalonFX(Constants.ARM_MOTOR_PORT),
                                           new CANSparkMax(Constants.INTAKE_ROLLER_PORT, MotorType.kBrushless));

    return intakeHardware;
  }

  /**
   * Hold arm up on init
   */
  public void initialize() {
    armUp();
  }

  /**
   * Create Shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Arm position (ticks)", () -> m_armMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Move arm to position
   * @param targetPosition position to move arm to
   */
  private void armSetPosition(final ArmPosition targetPosition) {
    m_armPosition = targetPosition;
    // Move arm toward setpoint
    m_armMotor.set(ControlMode.MotionMagic, m_armPosition.setPoint);
  }

  /**
   * Move arm to top position
   */
  void armUp() {
    armSetPosition(ArmPosition.Top);
  }

  /**
   * Move arm to bottom position
   */
  void armDown() {
    armSetPosition(ArmPosition.Bottom);
  }

   /**
   * Intake balls
   */
  void intake() {
    armDown();
    m_rollerMotor.setOpenLoopRampRate(0.1);
    m_rollerMotor.set(+m_rollerSpeed);
  }

  /**
   * Outtakes balls
   */
  void outtake() {
    armDown();
    m_rollerMotor.setOpenLoopRampRate(0.1);
    m_rollerMotor.set(-m_rollerSpeed);
  }

  /**
   * Stop roller and return arm to up/stowed position.
   */
  void stop() {
    m_rollerMotor.setOpenLoopRampRate(0.0);
    m_rollerMotor.stopMotor();
    armUp();
  }

  /**
   * Called to request that we start the intake of cargo.
   * 
   * @return true if the request was granted (currently retracted).
   */
  public boolean requestIntake() {
    return isStateMachineRunning()
        && isRetracted()
        && this.m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.INTAKE_REQUESTED);
  }

  /**
   * Called to request that we start the outtake of cargo.
   * 
   * @return true if the request was granted (currently retracted).
   */
  public boolean requestOuttake() {
    return isStateMachineRunning()
        && isRetracted()
        && this.m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.OUTTAKE_REQUESTED);
  }

  /**
   * Called to request that the intake be retracted.
   * 
   * @return true if the request was granted.
   */
  public boolean requestRetraction() {
    return isStateMachineRunning()
        && !isRetracted()
        && this.m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.RETRACTION_REQUESTED);
  }

  /**
   * @return true if intake has been requested but not yet handled.
   */
  boolean isIntakeRequested() {
    return this.m_request.get() == IntakeRequest.INTAKE_REQUESTED;
  }

  /**
   * @return true if intake has been requested but not yet handled.
   */
  boolean isOuttakeRequested() {
    return this.m_request.get() == IntakeRequest.OUTTAKE_REQUESTED;
  }

  /**
   * @return true if either intake or outake has been requested but not yet
   *         handled.
   */
  boolean isExtensionRequested() {
    return isIntakeRequested() || isOuttakeRequested();
  }

  /**
   * @return true if the intake is retracted or on its way to retracted and false
   *         otherwise.
   */
  boolean isRetracted() {
    return this.m_armPosition == ArmPosition.Top;
  }

  /**
   * @return true if intake retraction has been requested but not yet handled.
   */
  boolean isRetractionRequested() {
    return this.m_request.get() == IntakeRequest.RETRACTION_REQUESTED;
  }

  /**
   * @return true (the caller can move to next state machine state) if an
   *         intake had been requested.
   */
  boolean intakeHandled() {
    return this.m_request.compareAndSet(IntakeRequest.INTAKE_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
  }

  /**
   * @return true (the caller can move to next state machine state) if an
   *         outtake had been requested.
   */
  boolean outtakeHandled() {
    return this.m_request.compareAndSet(IntakeRequest.OUTTAKE_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
  }

  /**
   * @return true (the caller can move to next state machine state) if a
   *         retraction had been requested.
   */
  boolean retractionHandled() {
    return this.m_request.compareAndSet(IntakeRequest.RETRACTION_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
  }

  /**
   * @return true if the state machine is running and false otherwise.
   */
  public boolean isStateMachineRunning() {
    return this.m_stateMachine.getDefaultCommand().isScheduled();
  }

  @Override
  public void close() {
    m_armMotor = null;
    m_rollerMotor = null;
  }
}
