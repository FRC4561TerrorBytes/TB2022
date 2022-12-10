// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  /** Holds the fixed roller speed (TODO make this 2x drive speed) */
  private double m_rollerSpeed;
  /** The state machine running this intake. */
  private final IntakeStateMachine m_stateMachine;

  /**
   * Create an instance of IntakeSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param armConfig        PID config for arm
   * @param rollerSpeed      Intake roller speed [-1.0, +1.0]
   * @param rumbleController the driver controller to rumble as needed.
   */
  public IntakeSubsystem(final Hardware intakeHardware, final TalonPIDConfig armConfig, final double rollerSpeed,
      final GenericHID rumbleController) {
    this.m_armMotor = intakeHardware.armMotor;
    this.m_rollerMotor = intakeHardware.rollerMotor;
    this.m_armConfig = armConfig;
    this.m_rollerSpeed = rollerSpeed;

    m_rollerMotor.setIdleMode(IdleMode.kCoast);
    m_rollerMotor.setInverted(true);

    m_armConfig.initializeTalonPID(m_armMotor, FeedbackDevice.IntegratedSensor);
    m_armMotor.setNeutralMode(NeutralMode.Brake);
    m_armMotor.setSelectedSensorPosition(0.0);
    m_stateMachine = new IntakeStateMachine(this, rumbleController);
    setDefaultCommand(m_stateMachine.getDefaultCommand());
  }

  /**
   * Initialize hardware devices for intake subsystem
   * 
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
    tab.addString("Intake command", this::buildCommandString);
  }

  /**
   * @return a dashboard appropriate name for the current intake command.
   */
  private String buildCommandString() {
    final Command current = this.getCurrentCommand();
    if (current == null) {
      return "<null>";
    } else if (m_stateMachine.isStateMachineRunning()) {
      return "State machine: " + m_stateMachine.getCurrentState();
    }
    return current.getName();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Move arm to position
   * 
   * @param targetPosition position to move arm to
   */
  private void armSetPosition(final ArmPosition targetPosition) {
    // Move arm toward setpoint
    m_armMotor.set(ControlMode.MotionMagic, targetPosition.setPoint);
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
   * TODO dummy this up from dashboard and use for rumble.
   * 
   * @return true if the robot is holding the maximum number of cargo.
   */
  boolean isFull() {
    return false;
  }

  /**
   * Called to request that we start the intake of cargo.
   * 
   * @return true if the request was granted (currently retracted).
   */
  public boolean requestIntake() {
    return m_stateMachine.requestIntake();
  }

  /**
   * Called to request that we start the outtake of cargo.
   * 
   * @return true if the request was granted (currently retracted).
   */
  public boolean requestOuttake() {
    return m_stateMachine.requestOuttake();
  }

  /**
   * Called to request that the intake be retracted.
   * 
   * @return true if the request was granted.
   */
  public boolean requestRetraction() {
    return m_stateMachine.requestRetraction();
  }

  /**
   * <p>
   * This implementation is here to support the JUnit tests. The mock hardware
   * implementations are forgotten.
   * </p>
   * 
   * {@inheritDoc}
   */
  @Override
  public void close() {
    m_armMotor = null;
    m_rollerMotor = null;
  }
}
