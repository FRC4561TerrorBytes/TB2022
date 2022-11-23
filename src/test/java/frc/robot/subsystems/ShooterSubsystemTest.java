// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import frc.robot.Constants;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ShooterSubsystemTest {
  private final double DELTA = 2e-3;
  private ShooterSubsystem m_shooterSubsystem;
  private ShooterSubsystem.Hardware m_shooterHardware;

  private WPI_TalonFX m_flywheelMasterMotor, m_flywheelSlaveMotor, m_flywheelSmallMotor;
  private WPI_TalonSRX m_upperFeederMotor;
  private WPI_TalonSRX m_lowerFeederMotor;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_flywheelMasterMotor = mock(WPI_TalonFX.class);
    m_flywheelSlaveMotor = mock(WPI_TalonFX.class);
    m_flywheelSmallMotor = mock(WPI_TalonFX.class);
    m_upperFeederMotor = mock(WPI_TalonSRX.class);
    m_lowerFeederMotor = mock(WPI_TalonSRX.class);

    m_shooterHardware = new ShooterSubsystem.Hardware(m_flywheelMasterMotor, 
                                                      m_flywheelSlaveMotor, 
                                                      m_flywheelSmallMotor,
                                                      m_upperFeederMotor, 
                                                      m_lowerFeederMotor);

    m_shooterSubsystem = new ShooterSubsystem(m_shooterHardware,
                                              Constants.FLYWHEEL_MASTER_CONFIG,
                                              Constants.FLYWHEEL_SMALL_CONFIG,
                                              Constants.LOW_FLYWHEEL_SPEED,
                                              Constants.HIGH_FLYWHEEL_SPEED,
                                              Constants.FLYWHEEL_VISION_MAP,
                                              Constants.FEEDER_INTAKE_SPEED,
                                              Constants.FEEDER_SHOOT_SPEED);
  }

  @AfterEach
  public void close() {
    m_shooterSubsystem.close();
    m_shooterSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can set flywheel speed")
  public void setFlywheelSpeed() {
    m_shooterSubsystem.setFlywheelSpeed(200, 200);
    verify(m_flywheelMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.Velocity), AdditionalMatchers.eq(682.666, DELTA));
    verify(m_flywheelSmallMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.Velocity), AdditionalMatchers.eq(682.666, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can stop flywheel")
  public void stopFlywheel() {
    m_shooterSubsystem.flywheelStop();
    verify(m_flywheelMasterMotor, times(1)).stopMotor();
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can set feeder intake speed")
  public void feederIntake(){
    SensorCollection sensorCollection = mock(SensorCollection.class);
    when(m_upperFeederMotor.getSensorCollection()).thenReturn(sensorCollection);
    m_shooterSubsystem.feederIntake();
    verify(m_upperFeederMotor, times(1)).overrideLimitSwitchesEnable(true);
    verify(m_upperFeederMotor, times(1)).set(AdditionalMatchers.eq(Constants.FEEDER_INTAKE_SPEED, DELTA));
    verify(m_lowerFeederMotor, times(1)).set(AdditionalMatchers.eq(Constants.FEEDER_INTAKE_SPEED, DELTA));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can set feeder outtake speed")
  public void feederOuttake(){
    m_shooterSubsystem.feederOuttake();
    verify(m_upperFeederMotor, times(1)).overrideLimitSwitchesEnable(false);
    verify(m_upperFeederMotor, times(1)).set(AdditionalMatchers.eq(-Constants.FEEDER_INTAKE_SPEED, DELTA));
    verify(m_lowerFeederMotor, times(1)).set(AdditionalMatchers.eq(-Constants.FEEDER_INTAKE_SPEED, DELTA));

  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can set feeder shooter speed")
  public void feederShoot(){
    m_shooterSubsystem.feederShoot();
    verify(m_upperFeederMotor, times(1)).overrideLimitSwitchesEnable(false);
    verify(m_lowerFeederMotor, times(1)).set(AdditionalMatchers.eq(Constants.FEEDER_SHOOT_SPEED, DELTA));
    verify(m_upperFeederMotor, times(1)).set(AdditionalMatchers.eq(Constants.FEEDER_SHOOT_SPEED, DELTA));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can stop feeder motor")
  public void feederStop(){
    m_shooterSubsystem.feederStop(false);
    verify(m_upperFeederMotor, times(1)).stopMotor();
    verify(m_lowerFeederMotor, times(1)).stopMotor();
  }
}
