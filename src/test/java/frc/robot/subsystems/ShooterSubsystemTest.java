// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentCaptor;
import org.mockito.ArgumentMatchers;

import frc.robot.Constants;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ShooterSubsystemTest {
  private final double DELTA = 2e-3;
  private final double ALT_DELTA = 4e-3;
  private ShooterSubsystem m_shooterSubsystem;
  private ShooterSubsystem.Hardware m_shooterHardware;


  private WPI_TalonFX m_flywheelMasterMotor, m_flywheelSlaveMotor;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_flywheelMasterMotor = mock(WPI_TalonFX.class);
    m_flywheelSlaveMotor = mock(WPI_TalonFX.class);

    m_shooterHardware = new ShooterSubsystem.Hardware(m_flywheelMasterMotor, m_flywheelSlaveMotor);

    m_shooterSubsystem = new ShooterSubsystem(m_shooterHardware, Constants.FLYWHEEL_MASTER_CONFIG);
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
    m_shooterSubsystem.setFlywheelSpeed(200);
    verify(m_flywheelMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.Velocity), AdditionalMatchers.eq(682.666, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can stop flywheel")
  public void stopFlywheel() {
    m_shooterSubsystem.flywheelStop();
    verify(m_flywheelMasterMotor, times(1)).stopMotor();
  }
}