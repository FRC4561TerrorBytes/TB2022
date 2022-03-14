// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ClimberStateIterator;
import frc.robot.utils.TalonPIDConfig;

public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private WPI_TalonFX telescopeLeftMotor;
    private WPI_TalonFX telescopeRightMotor;
    private WPI_TalonFX winchMotor;

    public Hardware(WPI_TalonFX telescopeLeftMotor,
                    WPI_TalonFX telescopeRightMotor,
                    WPI_TalonFX winchMotor) {    
      this.telescopeLeftMotor = telescopeLeftMotor;
      this.telescopeRightMotor = telescopeRightMotor;
      this.winchMotor = winchMotor;
    }
  }

  private final String SUBSYSTEM_NAME = "Climber Subsystem";

  private WPI_TalonFX m_telescopeLeftMotor;
  private WPI_TalonFX m_telescopeRightMotor;
  private WPI_TalonFX m_winchMotor;
  private TalonPIDConfig m_telescopeConfig;
  private TalonPIDConfig m_winchConfig;
  private ClimberStateIterator m_climberStateIterator;
  private ClimberStateIterator.ClimberState m_currentState;

  private double m_leftTelescopePosition = 0.0;
  private double m_rightTelescopePosition = 0.0;

  /**
   * Creates an instance of ClimberSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!!!!!!
   * <p>
   * @param climberHardware Hardware devices required by climber 
   * @param telescopeConfig PID config for telescopes
   * @param winchConfig PID config for winch
   */
  public ClimberSubsystem(Hardware climberHardware, TalonPIDConfig telescopeConfig, TalonPIDConfig winchConfig) {
    this.m_telescopeLeftMotor = climberHardware.telescopeLeftMotor;
    this.m_telescopeRightMotor = climberHardware.telescopeRightMotor;
    this.m_winchMotor = climberHardware.winchMotor;
    this.m_telescopeConfig = telescopeConfig;
    this.m_winchConfig = winchConfig;
    this.m_climberStateIterator = new ClimberStateIterator(m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit(), m_winchConfig.getLowerLimit(), m_winchConfig.getUpperLimit());
    this.m_currentState = m_climberStateIterator.getCurrentState();

    m_telescopeConfig.initializeTalonPID(m_telescopeLeftMotor, FeedbackDevice.IntegratedSensor);
    m_telescopeLeftMotor.setSelectedSensorPosition(0.0);

    m_telescopeConfig.initializeTalonPID(m_telescopeRightMotor, FeedbackDevice.IntegratedSensor);
    m_telescopeRightMotor.setInverted(false);
    m_telescopeRightMotor.setSelectedSensorPosition(0.0);

    m_winchConfig.initializeTalonPID(m_winchMotor, FeedbackDevice.IntegratedSensor);
    m_winchMotor.setSelectedSensorPosition(0.0);

    m_telescopeLeftMotor.setNeutralMode(NeutralMode.Brake);
    m_telescopeRightMotor.setNeutralMode(NeutralMode.Brake);
    m_winchMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Initialize hardware devices for climber subsytem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climberHardware = new Hardware(new WPI_TalonFX(Constants.CLIMBER_LEFT_TELESCOPE_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.CLIMBER_RIGHT_TELESCOPE_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.CLIMBER_WINCH_MOTOR_PORT));

    return climberHardware;
  }

  /**
   * Hold telescope down and winch in on init
   */
  public void initialize() {
    telescopeSetPosition(m_telescopeLeftMotor, 0.0);
    telescopeSetPosition(m_telescopeRightMotor, 0.0);
    winchSetPosition(0.0);
  }

   /**
   * Create Shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Left telescope position", () -> m_telescopeLeftMotor.getSelectedSensorPosition());
    tab.addNumber("Left telescope target", () -> m_leftTelescopePosition);
    tab.addNumber("Right telescope position", () -> m_telescopeRightMotor.getSelectedSensorPosition());
    tab.addNumber("Right telescope target", () -> m_rightTelescopePosition);
    tab.addNumber("Winch position", () -> m_winchMotor.getSelectedSensorPosition());
  }

  /**
   * Create SmartDashboard indicators
   */
  public void smartDashboard() {
    SmartDashboard.putBoolean("Winch in?", m_winchMotor.getSelectedSensorPosition() < m_winchConfig.getTolerance());
  }

  @Override
  public void periodic() {
    smartDashboard();
  }

  /**
   * Moves climber to upper limit
   */
  public void telescopeUp() {
    telescopeSetPosition(m_telescopeLeftMotor, m_telescopeConfig.getUpperLimit());
    telescopeSetPosition(m_telescopeRightMotor, m_telescopeConfig.getUpperLimit());
  }

  /**
   * Moves climber to lower limit
   */
  public void telescopeDown() {
    telescopeSetPosition(m_telescopeLeftMotor, m_telescopeConfig.getLowerLimit());
    telescopeSetPosition(m_telescopeRightMotor, m_telescopeConfig.getLowerLimit());
  }

  /**
   * Move climber up relative to previous position
   */
  public void telescopeUpRelative() {
    m_leftTelescopePosition += 1000;
    m_rightTelescopePosition += 1000;
    m_leftTelescopePosition = MathUtil.clamp(m_leftTelescopePosition, m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit());
    m_rightTelescopePosition = MathUtil.clamp(m_rightTelescopePosition, m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit());
    telescopeSetPosition(m_telescopeLeftMotor, m_leftTelescopePosition);
    telescopeSetPosition(m_telescopeRightMotor, m_rightTelescopePosition);
  }

  /**
   * Move climber down relative to previous position
   */
  public void telescopeDownRelative() {
    m_leftTelescopePosition -= 1000;
    m_rightTelescopePosition -= 1000;
    m_leftTelescopePosition = MathUtil.clamp(m_leftTelescopePosition, m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit());
    m_rightTelescopePosition = MathUtil.clamp(m_rightTelescopePosition, m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit());
    telescopeSetPosition(m_telescopeLeftMotor, m_leftTelescopePosition);
    telescopeSetPosition(m_telescopeRightMotor, m_rightTelescopePosition);
  }

  /**
   * Stop telescopes after relative movement
   */
  public void telescopeStopRelative() {
    telescopeSetPosition(m_telescopeLeftMotor, m_leftTelescopePosition);
    telescopeSetPosition(m_telescopeRightMotor, m_rightTelescopePosition);
  }

  /**
   * Move telescopes up manually with percent output mode
   * @param speed motor speed in percent [0.0, +1.0]
   */
  public void telescopeManual(double speed) {
    m_telescopeLeftMotor.set(ControlMode.PercentOutput, speed);
    m_telescopeRightMotor.set(ControlMode.PercentOutput, speed);
  }

   /**
   * Move telescopes up manually with percent output mode
   * <p>
   * Disables soft limits
   * @param speed motor speed in percent [-1.0, +1.0]
   */
  public void telescopeManualOverride(double speed) {
    m_telescopeLeftMotor.overrideSoftLimitsEnable(false);
    m_telescopeRightMotor.overrideSoftLimitsEnable(false);
    m_telescopeLeftMotor.set(ControlMode.PercentOutput, speed);
    m_telescopeRightMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stop climber after moving manually, and re-enable soft limits
   */
  public void telescopeStop() {
    m_telescopeLeftMotor.stopMotor();
    m_telescopeRightMotor.stopMotor();
    m_telescopeLeftMotor.overrideSoftLimitsEnable(true);
    m_telescopeRightMotor.overrideSoftLimitsEnable(true);
  }

  /**
   * Moves telescope to set position.
   * @param position position in falcon ticks.
   */
  public void telescopeSetPosition(WPI_TalonFX talon, double position) {
    talon.set(ControlMode.MotionMagic, position);
  }

  /**
   * Moves winch to upper limit
   */
  public void winchIn() {
    winchSetPosition(m_winchConfig.getLowerLimit());
  }

  /**
   * Moves winch to lower limit
   */
  public void winchOut() {
    winchSetPosition(m_winchConfig.getUpperLimit());
  }

  /**
   * Move winch in manually
   * @param speed Winch motor speed [0.0, +1.0]
   */
  public void winchManual(double speed) {
    m_winchMotor.set(ControlMode.PercentOutput, MathUtil.clamp(speed, -1.0, 1.0));
  }

  /**
   * Move winch manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#winchStop()} to re-enable soft limits
   * @param speed motor speed in percent [-1.0, +1.0]
   */
  public void winchManualOverride(double speed) {
    m_winchMotor.overrideSoftLimitsEnable(false);
    m_winchMotor.set(ControlMode.PercentOutput, MathUtil.clamp(speed, -1.0, +1.0));
  }

  /**
   * Stop winch after moving manually and re-enable soft limits.
   */
  public void winchStop() {
    m_winchMotor.stopMotor();
    m_winchMotor.overrideSoftLimitsEnable(true);
  }

  /**
   * Moves winch to set position.
   * @param position position in falcon ticks.
   */
  public void winchSetPosition(double position) {
    position = MathUtil.clamp(position, m_winchConfig.getLowerLimit(), m_winchConfig.getUpperLimit());
    m_winchMotor.set(ControlMode.MotionMagic, position);
  }

  /**
   * Check if telesope motion is finished
   * @return true if telescope motion is complete
   */
  public boolean telescopeMotionIsFinished() {
    return m_telescopeRightMotor.getActiveTrajectoryPosition() - m_telescopeRightMotor.getClosedLoopTarget() < m_telescopeConfig.getTolerance();
  }

  /**
   * Check if winch motion is finished
   * @return true if winch motion is complete
   */
  public boolean winchMotionIsFinished() {
    return m_winchMotor.getActiveTrajectoryPosition() - m_winchMotor.getClosedLoopTarget() < m_winchConfig.getTolerance();
  }

  /**
   * Check if climber motion is finished
   * @return true if climber motion is complete
   */
  public boolean isClimbMotionFinished() {
    return telescopeMotionIsFinished() && winchMotionIsFinished();
  }

  /**
   * Advance climber to next climb state
   */
  public void nextClimberState() {
    m_climberStateIterator.nextState();
    m_currentState = m_climberStateIterator.getCurrentState();

    //telescopeSetPosition(m_telescopeLeftMotor, m_currentState.getTelescopePosition());
    telescopeSetPosition(m_telescopeRightMotor, m_currentState.getTelescopePosition());
    winchSetPosition(m_currentState.getWinchPosition());
  }

  /**
   * Move climber to previous state.
   */
  public void previousClimberState() {
    m_climberStateIterator.previousState();
    m_currentState = m_climberStateIterator.getCurrentState();

    //telescopeSetPosition(m_telescopeLeftMotor, m_currentState.getTelescopePosition());
    telescopeSetPosition(m_telescopeRightMotor, m_currentState.getTelescopePosition());
    winchSetPosition(m_currentState.getWinchPosition());
  }

  @Override
  public void close() {
    m_telescopeLeftMotor = null;
    m_telescopeRightMotor = null;
    m_winchMotor = null;
  }
}
