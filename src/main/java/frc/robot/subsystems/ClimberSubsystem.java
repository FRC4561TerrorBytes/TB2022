// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ClimberStateIterator;
import frc.robot.utils.TalonPIDConfig;
import frc.robot.utils.ClimberStateIterator.ClimberState;

public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {

  private final String SUBSYSTEM_NAME = "Climber Subsystem";
  
  public static class Hardware {
    private WPI_TalonFX telescopeMasterMotor;
    private WPI_TalonFX telescopSlaveMotor;
    private WPI_TalonFX winchMotor;
    private AnalogPotentiometer ultrasonicSensor;

    public Hardware(WPI_TalonFX telescopeMasterMotor,
                    WPI_TalonFX telescopeSlaveMotor,
                    WPI_TalonFX winchMotor,
                    AnalogPotentiometer ultrasonicSensor) {    
      this.telescopeMasterMotor = telescopeMasterMotor;
      this.telescopSlaveMotor = telescopeSlaveMotor;
      this.winchMotor = winchMotor;
      this.ultrasonicSensor = ultrasonicSensor;
    }
  }

  private final double ULTRASONIC_FACTOR = 512 / 39.37;

  private WPI_TalonFX m_telescopeMasterMotor;
  private WPI_TalonFX m_telescopeSlaveMotor;
  private WPI_TalonFX m_winchMotor;
  private AnalogPotentiometer m_ultrasonicSensor;
  private TalonPIDConfig m_telescopeConfig;
  private TalonPIDConfig m_winchConfig;
  private ClimberStateIterator m_climberStateIterator;
  private ClimberStateIterator.ClimberState m_currentState;


  /**
   * Creates an instance of ClimberSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!!!!!!
   * <p>
   * @param climberHardware Hardware devices required by climber 
   * @param telescopeConfig PID config for climber
   */
  public ClimberSubsystem(Hardware climberHardware, TalonPIDConfig telescopeConfig, TalonPIDConfig winchConfig) {
    this.m_telescopeMasterMotor = climberHardware.telescopeMasterMotor;
    this.m_telescopeSlaveMotor = climberHardware.telescopSlaveMotor;
    this.m_winchMotor = climberHardware.winchMotor;
    this.m_ultrasonicSensor = climberHardware.ultrasonicSensor;
    this.m_telescopeConfig = telescopeConfig;
    this.m_winchConfig = winchConfig;
    this.m_climberStateIterator = new ClimberStateIterator(Constants.TELESCOPE_UPPER_LIMIT, Constants.WINCH_UPPER_LIMIT);
    this.m_currentState = m_climberStateIterator.getCurrentState();

    m_telescopeConfig.initializeTalonPID(m_telescopeMasterMotor, FeedbackDevice.IntegratedSensor);
    m_telescopeMasterMotor.setSelectedSensorPosition(0.0);
    m_winchMotor.setSelectedSensorPosition(0.0);

    m_telescopeMasterMotor.setNeutralMode(NeutralMode.Brake);
    m_winchMotor.setNeutralMode(NeutralMode.Brake);

    m_telescopeSlaveMotor.set(ControlMode.Follower, m_telescopeMasterMotor.getDeviceID());
    m_telescopeSlaveMotor.setInverted(InvertType.OpposeMaster);
  }

  /**
   * Initialize hardware devices for intake subsytem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climberHardware = new Hardware(new WPI_TalonFX(Constants.CLIMBER_MASTER_TELESCOPE_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.CLIMBER_SLAVE_TELESCOPE_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.CLIMBER_WINCH_MOTOR_PORT),
                                            new AnalogPotentiometer(Constants.CLIMBER_ULTRASONIC_PORT));

    return climberHardware;
  }

  /**
   * Hold telescope down and winch in on init
   */
  public void initialize() {
    telescopeSetPosition(m_telescopeConfig.getLowerLimit());
    winchSetPosition(m_winchConfig.getLowerLimit());
  }

   /**
   * Create Shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Climber position", () -> m_telescopeMasterMotor.getSelectedSensorPosition());
    tab.addNumber("Winch position", () -> m_winchMotor.getSelectedSensorPosition());
  }

  /**
   * Create SmartDashboard indicators
   */
  public void smartDashboard() {
    SmartDashboard.putBoolean("Climber Ready?", isClimbMotionFinished());
  }

  @Override
  public void periodic() {
    smartDashboard();
  }

  public double getUltrasonicDistance() {
    return m_ultrasonicSensor.get() * ULTRASONIC_FACTOR;
  }

  /**
   * Moves climber to upper limit
   */
  public void telescopeUp() {
    telescopeSetPosition(m_telescopeConfig.getUpperLimit());
  }

  /**
   * Moves climber to lower limit
   */
  public void telescopeDown() {
    telescopeSetPosition(m_telescopeConfig.getLowerLimit());
  }

  /**
   * Move climber up manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#telescopeStopManual()} to re-enable soft limits
   */
  public void telescopeUpManual() {
    m_telescopeMasterMotor.overrideSoftLimitsEnable(false);
    m_telescopeMasterMotor.set(ControlMode.PercentOutput, +1.0);
  }

  /**
   * Move climber down manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#telescopeStopManual()} to re-enable soft limits
   */
  public void telescopeDownManual() {
    m_telescopeMasterMotor.overrideSoftLimitsEnable(false);
    m_telescopeMasterMotor.set(ControlMode.PercentOutput, -1.0);
  }

  /**
   * Stop climber after moving manually, and re-enable soft limits
   */
  public void telescopeStopManual() {
    m_telescopeMasterMotor.overrideSoftLimitsEnable(true);
    m_telescopeMasterMotor.stopMotor();
  }

  /**
   * Moves telescope to set position.
   * @param position position in falcon ticks.
   */
  public void telescopeSetPosition(double position) {
    position = MathUtil.clamp(position, m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit());
    m_telescopeMasterMotor.set(ControlMode.MotionMagic, position);
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
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#winchStopManual()} to re-enable soft limits
   */
  public void winchInManual() {
    m_winchMotor.overrideSoftLimitsEnable(false);
    m_winchMotor.set(ControlMode.PercentOutput, -1.0);
  }

  /**
   * Move winch out manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#winchStopManual()} to re-enable soft limits
   */
  public void winchOutManual() {
    m_winchMotor.overrideSoftLimitsEnable(false);
    m_winchMotor.set(ControlMode.PercentOutput, +1.0);
  }

  /**
   * Stop winch after moving manually and re-enable soft limits.
   */
  public void winchStopManual() {
    m_winchMotor.overrideSoftLimitsEnable(true);
    m_winchMotor.stopMotor();
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
   return m_telescopeMasterMotor.getActiveTrajectoryPosition() - m_telescopeMasterMotor.getClosedLoopTarget() < m_telescopeConfig.getTolerance();
  }

  /**
   * Check if winch motion is finished
   * @return true if winch motion is complete
   */
  public boolean winchMotionIsFinished() {
    m_winchMotor.getClosedLoopError();
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

    telescopeSetPosition(m_currentState.getTelescopePosition());
    winchSetPosition(m_currentState.getWinchPosition());
  }

  /**
   * Move climber to previous state.
   */
  public void previousClimberState() {
    m_climberStateIterator.previousState();
    m_currentState = m_climberStateIterator.getCurrentState();

    telescopeSetPosition(m_currentState.getTelescopePosition());
    winchSetPosition(m_currentState.getWinchPosition());
  }

  @Override
  public void close() {
    m_telescopeMasterMotor = null;
    m_telescopeSlaveMotor = null;
    m_winchMotor = null;
  }
}
