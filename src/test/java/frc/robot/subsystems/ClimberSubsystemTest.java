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

/** Add your docs here. */
public class ClimberSubsystemTest {

    private final double DELTA = 1e-3;
    private ClimberSubsystem m_climberSubsystem;
    private ClimberSubsystem.Hardware m_climberHardware;

    private WPI_TalonFX m_climberMotor;

    public void setup() {
		
		// Create mock harware device
		m_climberMotor = mock(WPI_TalonFX.class);

		// Create Hardware objects using mock objects
		m_climberHardware = new ClimberSubsystem.Hardware(m_climberMotor);

		// Create intakeSubsystem object
		m_climberSubsystem = new ClimberSubsystem(m_climberHardware, Constants.CLIMBER_CONFIG);
	}

    @AfterEach
	public void close() {
		m_climberSubsystem.close();
		m_climberSubsystem = null;
	}

    public void climberUp(){
        m_climberSubsystem.climberUp();
    }

    public void climberDown(){
        m_climberSubsystem.climberDown();
    }
}
