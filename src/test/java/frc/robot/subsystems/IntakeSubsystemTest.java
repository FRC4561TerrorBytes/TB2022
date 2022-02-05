// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

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
public class IntakeSubsystemTest {
	private final double DELTA = 1e-3;
	private IntakeSubsystem m_intakeSubsystem;
	private IntakeSubsystem.Hardware m_intakeHardware;

	private WPI_TalonFX m_armMotor;
	private CANSparkMax m_rollerMotor;

	@BeforeEach
	public void setup() {
		// Create mock harware device
		m_armMotor = mock(WPI_TalonFX.class);
		m_rollerMotor = mock(CANSparkMax.class);

		// Create Hardware objects using mock objects
		m_intakeHardware = new IntakeSubsystem.Hardware(m_armMotor, m_rollerMotor);

		// Create intakeSubsystem object
		m_intakeSubsystem = new IntakeSubsystem(m_intakeHardware, Constants.INTAKE_ARM_CONFIG, Constants.INTAKE_ROLLER_SPEED);
	}

	@AfterEach
	public void close() {
		m_intakeSubsystem.close();
		m_intakeSubsystem = null;
	}

	@Test
	@Order(1)
	@DisplayName("Test if robot can toggle up or down")
	public void toggleArm() {
		m_intakeSubsystem.toggleArmPosition();
		verify(m_armMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_LOWER_LIMIT, DELTA));

		m_intakeSubsystem.toggleArmPosition();
		verify(m_armMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_UPPER_LIMIT, DELTA));
	}

	@Test
	@Order(2)
	@DisplayName("Test if robot can move arm relative to previous setpoint")
	public void armRelative() {
		when(m_armMotor.getClosedLoopTarget()).thenReturn(-42.0);

		m_intakeSubsystem.armPositionRelative(4.0);
		verify(m_armMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(-38.0, DELTA));
	}

	@Test
	@Order(3)
	@DisplayName("Test if robot can intake")
	public void intake() {
		m_intakeSubsystem.intake();
		verify(m_armMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_LOWER_LIMIT, DELTA));
		verify(m_rollerMotor, times(1)).set(AdditionalMatchers.eq(Constants.INTAKE_ROLLER_SPEED, DELTA));
	}

	@Test
	@Order(4)
	@DisplayName("Test if robot can outtake")
	public void outtake() {
		m_intakeSubsystem.outtake();
		verify(m_rollerMotor, times(1)).set(AdditionalMatchers.eq(-Constants.INTAKE_ROLLER_SPEED, DELTA));
	}

	@Test
	@Order(5)
	@DisplayName("Test if robot can stop roller while intaking and return arm to up position")
	public void stopIntake() {
		m_intakeSubsystem.armSetPosition(IntakeSubsystem.ArmPosition.Top.value);
		m_intakeSubsystem.intake();
		m_intakeSubsystem.stop();
		verify(m_armMotor, times(2)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_UPPER_LIMIT, DELTA));
		verify(m_armMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_LOWER_LIMIT, DELTA));
		verify(m_rollerMotor, times(1)).set(AdditionalMatchers.eq(Constants.INTAKE_ROLLER_SPEED, DELTA));
		verify(m_rollerMotor, times(1)).stopMotor();
	}

	@Test
	@Order(6)
	@DisplayName("Test if robot can stop roller when arm is down and keep it down")
	public void stopIntakeArmDown() {
		m_intakeSubsystem.armSetPosition(IntakeSubsystem.ArmPosition.Bottom.value);
		m_intakeSubsystem.intake();
		m_intakeSubsystem.stop();
		verify(m_armMotor, times(3)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_LOWER_LIMIT, DELTA));
		verify(m_rollerMotor, times(1)).set(AdditionalMatchers.eq(Constants.INTAKE_ROLLER_SPEED, DELTA));
		verify(m_rollerMotor, times(1)).stopMotor();
	}

	@Test
	@Order(7)
	@DisplayName("Test if robot can stop roller while outtaking and return arm to up position")
	public void stopOuttake() {
		m_intakeSubsystem.armSetPosition(IntakeSubsystem.ArmPosition.Top.value);
		m_intakeSubsystem.outtake();
		m_intakeSubsystem.stop();
		verify(m_armMotor, times(2)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_UPPER_LIMIT, DELTA));
		verify(m_armMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_LOWER_LIMIT, DELTA));
		verify(m_rollerMotor, times(1)).set(AdditionalMatchers.eq(-Constants.INTAKE_ROLLER_SPEED, DELTA));
		verify(m_rollerMotor, times(1)).stopMotor();
	}

	@Test
	@Order(8)
	@DisplayName("Test if robot can stop roller while outtaking when arm is down and keep it down")
	public void stopOuttakeArmDown() {
		m_intakeSubsystem.armSetPosition(IntakeSubsystem.ArmPosition.Bottom.value);
		m_intakeSubsystem.outtake();
		m_intakeSubsystem.stop();
		verify(m_armMotor, times(3)).set(ArgumentMatchers.eq(ControlMode.MotionMagic), AdditionalMatchers.eq(Constants.INTAKE_ARM_LOWER_LIMIT, DELTA));
		verify(m_rollerMotor, times(1)).set(AdditionalMatchers.eq(-Constants.INTAKE_ROLLER_SPEED, DELTA));
		verify(m_rollerMotor, times(1)).stopMotor();
	}
	
}
