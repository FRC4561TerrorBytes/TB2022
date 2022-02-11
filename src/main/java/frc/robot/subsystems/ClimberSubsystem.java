// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Iterator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonPIDConfig;

public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {

  private final String SUBSYSTEM_NAME = "Climber Subsystem";
  
  public static class Hardware {
    private WPI_TalonFX telescopeMasterMotor;
    private WPI_TalonFX telescopSlaveMotor;
    private WPI_TalonFX winchMotor;
    private AnalogPotentiometer ultrasonicSensor;

    public Hardware(WPI_TalonFX winchMotor,
                    WPI_TalonFX telescopeMasterMotor,
                    WPI_TalonFX telescopeSlaveMotor,
                    AnalogPotentiometer ultrasonicSensor) {    
      this.telescopeMasterMotor = telescopeMasterMotor;
      this.telescopSlaveMotor = telescopeSlaveMotor;
      this.winchMotor = winchMotor;
      this.ultrasonicSensor = ultrasonicSensor;
    }
  }

  public static enum ClimberState implements Iterator {
    climberStart(0, 0, 0),
    telescopeUp(1, 0, 0),
    telescopeDown(2, 0, 0),
    telescopeUnhook(3, 0, 0),
    winchOut(4, 0, 0),
    telescopeReach(5, 0, 0),
    winchHook(6, 0, 0),
    telescopeGrab(7, 0, 0),
    robotSwing(8, 0, 0),
    winchIn(9, 0, 0),
    finishClimb(10, 0, 0);

    private final int step;
    private final int telescopeRotations;
    private final int winchRotations;
    private ClimberState(int step, int telescopeRotations, int winchRotations) {
      this.step = step;
      this.telescopeRotations = telescopeRotations;
      this.winchRotations = winchRotations;
    }

    @Override
    public boolean hasNext() {
      // TODO Auto-generated method stub
      return false;
    }

    @Override
    public Object next() {
      // TODO Auto-generated method stub
      return null;
    }
  }

  private final double ULTRASONIC_FACTOR = 512 / 39.37;

  private WPI_TalonFX m_telescopeMasterMotor;
  private WPI_TalonFX m_telescopeSlaveMotor;
  private WPI_TalonFX m_winchMotor;
  private AnalogPotentiometer m_ultrasonicSensor;
  private TalonPIDConfig m_telescopeConfig;


  /**
   * Creates an instance of ClimberSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!!!!!!
   * <p>
   * @param climberHardware Hardware devices required by climber 
   * @param telescopeConfig PID config for climber
   */
  public ClimberSubsystem(Hardware climberHardware, TalonPIDConfig telescopeConfig) {
    this.m_telescopeMasterMotor = climberHardware.telescopeMasterMotor;
    this.m_telescopeSlaveMotor = climberHardware.telescopSlaveMotor;
    this.m_winchMotor = climberHardware.winchMotor;
    this.m_ultrasonicSensor = climberHardware.ultrasonicSensor;
    this.m_telescopeConfig = telescopeConfig;

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

  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Climber position", () -> m_telescopeMasterMotor.getSelectedSensorPosition());
    tab.addNumber("Winch position", () -> m_winchMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getUltrasonicDistance() {
    return m_ultrasonicSensor.get() * ULTRASONIC_FACTOR;
  }

  /**
   * Moves climber to upper limit
   */
  public void climberUp() {
    m_telescopeMasterMotor.set(ControlMode.MotionMagic, m_telescopeConfig.getUpperLimit());
  }

  /**
   * Moves climber to lower limit
   */
  public void climberDown() {
    m_telescopeMasterMotor.set(ControlMode.MotionMagic, m_telescopeConfig.getLowerLimit());
  }

  /**
   * Move climber up manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#climberStopManual()} to re-enable soft limits
   */
  public void climberUpManual() {
    m_telescopeMasterMotor.overrideSoftLimitsEnable(false);
    m_telescopeMasterMotor.set(ControlMode.PercentOutput, +1.0);
  }

  /**
   * Move climber down manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#climberStopManual()} to re-enable soft limits
   */
  public void climberDownManual() {
    m_telescopeMasterMotor.overrideSoftLimitsEnable(false);
    m_telescopeMasterMotor.set(ControlMode.PercentOutput, -1.0);
  }

  /**
   * Stop climber after moving manually, and re-enable soft limits
   */
  public void climberStopManual() {
    m_telescopeMasterMotor.overrideSoftLimitsEnable(true);
    m_telescopeMasterMotor.stopMotor();
  }

  /**
   * Moves winch to upper limit
   */
  public void winchUp() {
    m_winchMotor.set(ControlMode.PercentOutput, m_telescopeConfig.getUpperLimit());
  }

  /**
   * Moves winch to lower limit
   */
  public void winchDown() {
    m_winchMotor.set(ControlMode.PercentOutput, m_telescopeConfig.getLowerLimit());
  }

  @Override
  public void close() {
    m_telescopeMasterMotor = null;
    m_telescopeSlaveMotor = null;
    m_winchMotor = null;
  }
}
