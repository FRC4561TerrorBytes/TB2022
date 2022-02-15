// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

import edu.wpi.first.wpilibj.Counter;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ShootCommandTest {
  private final double DELTA = 2e-3;
  private ShooterSubsystem m_shooterSubsystem;
  private ShootCommand m_shootCommand;
  private ShooterSubsystem.Hardware m_shooterHardware;


  private WPI_TalonFX m_flywheelMasterMotor, m_flywheelSlaveMotor;
  private WPI_TalonSRX m_upperFeederMotor;
  private WPI_TalonSRX m_lowerFeederMotor;
  private Counter m_lidar;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_flywheelMasterMotor = mock(WPI_TalonFX.class);
    m_flywheelSlaveMotor = mock(WPI_TalonFX.class);
    m_upperFeederMotor = mock(WPI_TalonSRX.class);
    m_lowerFeederMotor = mock(WPI_TalonSRX.class);
    m_lidar = mock(Counter.class);

    m_shooterHardware = new ShooterSubsystem.Hardware(m_flywheelMasterMotor, m_flywheelSlaveMotor, m_upperFeederMotor, m_lowerFeederMotor, m_lidar);

    m_shooterSubsystem = new ShooterSubsystem(m_shooterHardware, Constants.FLYWHEEL_MASTER_CONFIG);
    m_shootCommand = new ShootCommand(m_shooterSubsystem, 200.0);
  }

  @AfterEach
  public void close() {
    m_shooterSubsystem.close();
    m_shooterSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can start flywheel")
  public void initialize() {
      m_shootCommand.initialize();
    verify(m_flywheelMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.Velocity), AdditionalMatchers.eq(682.666, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can run feeder wheel when flywheel at speed")
  public void executeMotorAtSpeed() {
    m_shootCommand.initialize();
    when(m_flywheelMasterMotor.getClosedLoopError()).thenReturn(0.0);
    when(m_flywheelMasterMotor.getClosedLoopTarget()).thenReturn(682.666);
    m_shootCommand.execute();
    verify(m_upperFeederMotor, times(1)).overrideLimitSwitchesEnable(false);
    verify(m_lowerFeederMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot stops feeder when flywheel not at speed")
  public void executeMotorNotAtSpeed(){
    m_shootCommand.initialize();
    when(m_flywheelMasterMotor.getClosedLoopError()).thenReturn(150.0);
    when(m_flywheelMasterMotor.getClosedLoopTarget()).thenReturn(682.666);
    m_shootCommand.execute();
    verify(m_upperFeederMotor, times(1)).overrideLimitSwitchesEnable(true);
    verify(m_lowerFeederMotor, times(1)).stopMotor();
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can stop feeder wheel and flywheel")
  public void end(){
    m_shootCommand.end(true);
    verify(m_flywheelMasterMotor, times(1)).stopMotor();
    verify(m_upperFeederMotor, times(1)).overrideLimitSwitchesEnable(true);
    verify(m_lowerFeederMotor, times(1)).stopMotor();
    
  }
}
