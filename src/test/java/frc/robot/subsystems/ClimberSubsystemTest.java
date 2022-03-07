// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
public class ClimberSubsystemTest {
	
	private final double DELTA = 1e-3;
	private ClimberSubsystem m_climberSubsystem;
	private ClimberSubsystem.Hardware m_climberHardware;
	
	private WPI_TalonFX m_telescopeMasterMotor;
	private WPI_TalonFX m_telescopeSlaveMotor;
	private WPI_TalonFX m_winchMotor;
	
	@BeforeEach
	public void setup() {
		// Create mock harware device
		m_telescopeMasterMotor = mock(WPI_TalonFX.class);
		m_telescopeSlaveMotor = mock(WPI_TalonFX.class);
		m_winchMotor = mock(WPI_TalonFX.class);
		
		// Create Hardware objects using mock objects
		m_climberHardware = new ClimberSubsystem.Hardware(m_telescopeMasterMotor, m_telescopeSlaveMotor, m_winchMotor);
		
		// Create ClimberSubsystem object
		m_climberSubsystem = new ClimberSubsystem(m_climberHardware, Constants.TELESCOPE_CONFIG, Constants.WINCH_CONFIG);
	}
	
	@AfterEach
	public void close() {
		m_climberSubsystem.close();
		m_climberSubsystem = null;
	}
	
	@Test
	@Order(1)
	@DisplayName("Test if robot can move climber up automatically")
	public void climberUp() {
		m_climberSubsystem.telescopeUp();
		verify(m_telescopeMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.TELESCOPE_UPPER_LIMIT, DELTA));
	}
	
	@Test
	@Order(2)
	@DisplayName("Test if robot can move climber down automatically")
	public void climberDown(){
		m_climberSubsystem.telescopeDown();
		verify(m_telescopeMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.TELESCOPE_LOWER_LIMIT, DELTA));
	}

	@Test
	@Order(3)
	@DisplayName("Test if robot can move climber up manually")
	public void climberUpManual() {
		m_climberSubsystem.telescopeUpManual();
		verify(m_telescopeMasterMotor, times(1)).overrideSoftLimitsEnable(ArgumentMatchers.eq(false));
		verify(m_telescopeMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.5, DELTA));

		m_climberSubsystem.telescopeStopManual();
		verify(m_telescopeMasterMotor, times(1)).overrideSoftLimitsEnable(ArgumentMatchers.eq(false));
		verify(m_telescopeMasterMotor, times(1)).stopMotor();
	}

	@Test
	@Order(4)
	@DisplayName("Test if robot can move climber down manually")
	public void climberDownManual() {
		m_climberSubsystem.telescopeDownManual();
		verify(m_telescopeMasterMotor, times(1)).overrideSoftLimitsEnable(ArgumentMatchers.eq(false));
		verify(m_telescopeMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(-0.5, DELTA));

		m_climberSubsystem.telescopeStopManual();
		verify(m_telescopeMasterMotor, times(1)).overrideSoftLimitsEnable(ArgumentMatchers.eq(false));
		verify(m_telescopeMasterMotor, times(1)).stopMotor();
	}

	@Test
	@Order(5)
	@DisplayName("Test if robot can stop moving manually")
	public void climberStopManual() {
		m_climberSubsystem.telescopeStopManual();
		verify(m_telescopeMasterMotor, times(1)).overrideSoftLimitsEnable(ArgumentMatchers.eq(true));
		verify(m_telescopeMasterMotor, times(1)).stopMotor();
	}
}
