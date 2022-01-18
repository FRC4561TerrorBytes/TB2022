// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;

public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {

  private final String SUBSYSTEM_NAME = "Climber Subsystem";
  
  public static class Hardware {
    private WPI_TalonFX climberMotor;

    public Hardware(WPI_TalonFX climberMotor) {
      this.climberMotor = climberMotor;
    }
  }

  private WPI_TalonFX m_climberMotor;
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
    this.m_climberConfig = climberConfig;
  }

  /**
   * Initialize hardware devices for intake subsytem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climberHardware = new Hardware(new WPI_TalonFX(Constants.CLIMBER_MOTOR_PORT));

    return climberHardware;
  }

  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Climber position", () -> m_climberMotor.getSelectedSensorPosition());
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
   * Manual climber control
   * @param speed Speed percentage [-1, 1]
   */
  public void climberManual(double speed) {
    m_climberMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void close() {
    m_climberMotor = null;
  }
}
