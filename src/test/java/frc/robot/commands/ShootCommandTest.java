// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import edu.wpi.first.wpilibj.Counter;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class ShootCommandTest {
    private final double DELTA = 2e-3;
    private static ShooterSubsystem m_shooterSubsystem;
    private static double m_rpm;
    private static ShootCommand m_shootCommand;
    private static ShooterSubsystem.Hardware m_shooterHardware;
    private static WPI_TalonFX m_flywheelMasterMotor; 
    private static WPI_TalonFX m_flywheelSlaveMotor; 
    private static WPI_TalonFX m_feederMotor;
    private static Counter m_lidar;

    @BeforeAll
    public static void setup() {
        m_shooterSubsystem = mock(ShooterSubsystem.class);
        m_flywheelMasterMotor = mock(WPI_TalonFX.class); 
        m_flywheelSlaveMotor = mock(WPI_TalonFX.class); 
        m_feederMotor = mock(WPI_TalonFX.class);
        m_lidar = mock(Counter.class);

        m_shooterHardware = new ShooterSubsystem.Hardware(m_flywheelMasterMotor, 
                                                          m_flywheelSlaveMotor, 
                                                          m_feederMotor,
                                                          m_lidar);

        m_shooterSubsystem = new ShooterSubsystem(m_shooterHardware, Constants.FLYWHEEL_MASTER_CONFIG);

        m_shootCommand = new ShootCommand(m_shooterSubsystem, Constants.FLYWHEEL_SHOOTING_RPM);
    }

    @AfterAll
    public static void close() {
        m_shooterSubsystem.close();
        m_shooterSubsystem = null;
    }

    @Test
    @Order(1)
    @DisplayName("Test if flywheel gets up to speed")
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

    @Test
    @Order(3)
    @DisplayName("Test if feeder can intake")
    public void feederIntake() {
        m_shooterSubsystem.feederIntake();
        
        verify(m_feederMotor, times(1)).set(Constants.FEEDER_INTAKE_SPEED);
    }

    @Test
    @Order(4)
    @DisplayName("Test if feeder can outtake")
    public void feederOuttake() {
        m_shooterSubsystem.feederOuttake();
        
        verify(m_feederMotor, times(1)).set(-Constants.FEEDER_INTAKE_SPEED);
    }

    @Test
    @Order(5)
    @DisplayName("Test if robot can stop feeder motor")
    public void stopFeeder() {
        m_shooterSubsystem.feederStop();
        verify(m_feederMotor, times(1)).stopMotor();
    }
}
