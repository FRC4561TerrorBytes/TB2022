// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonPIDConfig;

public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {

  private final String SUBSYSTEM_NAME = "Climber Subsystem";
  
  public static class Hardware {
    private WPI_TalonFX climberMotor;
    private WPI_TalonFX winchMotor;

    public Hardware(WPI_TalonFX climberMotor, WPI_TalonFX winchMotor) {
      this.climberMotor = climberMotor;
      this.winchMotor = winchMotor;
    }
  }

  private WPI_TalonFX m_climberMotor;
  private WPI_TalonFX m_winchMotor;
  private TalonPIDConfig m_climberConfig;

  /**
   * Creates an instance of ClimberSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!!!!!!
   * <p>
   * @param climberHardware Hardware devices required by climber 
   * @param climberConfig PID config for climber
   */
  public ClimberSubsystem(Hardware climberHardware, TalonPIDConfig climberConfig) {
    this.m_climberMotor = climberHardware.climberMotor;
    this.m_winchMotor = climberHardware.winchMotor;
    this.m_climberConfig = climberConfig;

    m_climberConfig.initializeTalonPID(m_climberMotor, FeedbackDevice.IntegratedSensor);
    m_climberMotor.setSelectedSensorPosition(0.0);
    m_winchMotor.setSelectedSensorPosition(0.0);

    m_climberMotor.setNeutralMode(NeutralMode.Brake);
    m_winchMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Initialize hardware devices for intake subsytem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climberHardware = new Hardware(new WPI_TalonFX(Constants.CLIMBER_MOTOR_PORT), 
                                            new WPI_TalonFX(Constants.CLIMBER_WINCH_MOTOR_PORT));

    return climberHardware;
  }

  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Climber position", () -> m_climberMotor.getSelectedSensorPosition());
    tab.addNumber("Winch position", () -> m_winchMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Moves climber to upper limit
   */
  public void climberUp() {
    m_climberMotor.set(ControlMode.MotionMagic, m_climberConfig.getUpperLimit());
  }

  /**
   * Moves climber to lower limit
   */
  public void climberDown() {
    m_climberMotor.set(ControlMode.MotionMagic, m_climberConfig.getLowerLimit());
  }

  /**
   * Move climber up manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#climberStopManual()} to re-enable soft limits
   */
  public void climberUpManual() {
    m_climberMotor.overrideSoftLimitsEnable(false);
    m_climberMotor.set(ControlMode.PercentOutput, +1.0);
  }

  /**
   * Move climber down manually
   * <p>
   * Note: soft limits are disabled, call {@link ClimberSubsystem#climberStopManual()} to re-enable soft limits
   */
  public void climberDownManual() {
    m_climberMotor.overrideSoftLimitsEnable(false);
    m_climberMotor.set(ControlMode.PercentOutput, -1.0);
  }

  /**
   * Stop climber after moving manually, and re-enable soft limits
   */
  public void climberStopManual() {
    m_climberMotor.overrideSoftLimitsEnable(true);
    m_climberMotor.stopMotor();
  }

  /**
   * Moves winch to upper limit
   */
  public void winchUp() {
    m_winchMotor.set(ControlMode.PercentOutput, m_climberConfig.getUpperLimit());
  }

  /**
   * Moves winch to lower limit
   */
  public void winchDown() {
    m_winchMotor.set(ControlMode.PercentOutput, m_climberConfig.getLowerLimit());
  }

  @Override
  public void close() {
    m_climberMotor = null;
    m_winchMotor = null;
  }
}
