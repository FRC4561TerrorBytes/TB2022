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
public class DriveSubsystemTest {

  private final double DELTA = 2e-3;
  private final double ALT_DELTA = 5e-2;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private WPI_TalonFX m_lMasterMotor;
  private WPI_TalonFX m_rMasterMotor;
  private WPI_TalonFX m_leftSlaveMotor;
  private WPI_TalonFX m_rightSlaveMotor;
  
  private AHRS m_navx;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_lMasterMotor = mock(WPI_TalonFX.class);
    m_rMasterMotor = mock(WPI_TalonFX.class);
    m_leftSlaveMotor = mock(WPI_TalonFX.class);
    m_rightSlaveMotor = mock(WPI_TalonFX.class);
    m_navx = mock(AHRS.class);

    // Create Hardware object using mock objects
    m_drivetrainHardware = new DriveSubsystem.Hardware(m_lMasterMotor, m_rMasterMotor, m_leftSlaveMotor, m_rightSlaveMotor, m_navx);

    // Create DriveSubsystem object
    m_driveSubsystem = new DriveSubsystem(m_drivetrainHardware, 
                                          Constants.DRIVE_kP,
                                          Constants.DRIVE_kD, 
                                          Constants.DRIVE_TURN_SCALAR,
                                          Constants.CONTROLLER_DEADBAND,
                                          Constants.DRIVE_METERS_PER_TICK,
                                          Constants.DRIVE_MAX_LINEAR_SPEED,
                                          Constants.DRIVE_SLIP_RATIO,
                                          Constants.DRIVE_TRACTION_CONTROL_CURVE,
                                          Constants.DRIVE_THROTTLE_INPUT_CURVE);
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can move forward using PID drive")
  public void forward() {
    // Hardcode NAVX sensor return values for angle, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)3.766);

    // Try to drive forward
    m_driveSubsystem.teleopPID(1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can move in reverse using PID drive")
  public void reverse() {
    // Hardcode NAVX sensor return values for angle, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)-3.766);

    // Try to drive in reverse
    m_driveSubsystem.teleopPID(-1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(-1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(-1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can reverse direction using PID drive")
  public void reverseDirection() {
    // Initial velocity
    double velocity = 1.883;
    when(m_navx.getVelocityY()).thenReturn((float)velocity);

    // Argument capture objects
    ArgumentCaptor<Double> lMotorOutputs = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rMotorOutputs = ArgumentCaptor.forClass(Double.class);

    // Hardcode NAVX sensor value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Simulate NAVX sensor deceleration
    for (int i = 0; i < 180; i++) {
      velocity = (velocity <= -1.883) ?
                  -1.883 :
                  velocity - 0.03;
      when(m_navx.getVelocityY()).thenReturn((float)velocity);

      // Try to reverse direction
      m_driveSubsystem.teleopPID(-0.5, 0.0);
      m_driveSubsystem.periodic();
    }

    // Verify that left and right motors are being driven and capture output
    verify(m_lMasterMotor, times(180)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), lMotorOutputs.capture(), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(180)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), rMotorOutputs.capture(), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));

    // Check that last motor output value was as expected
    double lMotorOutput = lMotorOutputs.getAllValues().get(lMotorOutputs.getAllValues().size() - 1);
    double rMotorOutput = rMotorOutputs.getAllValues().get(rMotorOutputs.getAllValues().size() - 1);
    assertTrue(Math.abs(-0.5 - lMotorOutput) <= ALT_DELTA);
    assertTrue(Math.abs(-0.5 - rMotorOutput) <= ALT_DELTA);
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can stop using PID drive")
  public void stop() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to stop
    m_driveSubsystem.teleopPID(0.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot ignores small throttle input values under threshold")
  public void ignoreSmallThrottleInput() {
    // Hardcode NAVX sensor return values for angle, velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to drive with small throttle value
    m_driveSubsystem.teleopPID(0.007, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA),
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));                  
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot ignores small turn input values under threshold")
  public void ignoreSmallTurnInput() {
    // Hardcode NAVX sensor return values for angle, velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)3.766);

    // Try to drive with small turn value
    m_driveSubsystem.teleopPID(1.0, 0.004);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(7)
  @DisplayName("Test if robot can turn left using PID drive")
  public void turningLeft() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to turn left
    m_driveSubsystem.teleopPID(0.0, -1.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.gt(0.0));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.lt(0.0));
  }

  @Test
  @Order(8)
  @DisplayName("Test if robot can turn right using PID drive")
  public void turningRight() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to turn right
    m_driveSubsystem.teleopPID(0.0, 1.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.lt(0.0));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.gt(0.0));
  }

  @Test
  @Order(9)
  @DisplayName("Test if robot will limit wheel slip")
  public void tractionControl() {
    // Hardcode NAVX sensor values for angle, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Enable traction control
    m_driveSubsystem.enableTractionControl();

    // Try to move forward
    m_driveSubsystem.teleopPID(1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.lt(Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.gt(0.0)), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.lt(Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.gt(0.0)), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));

    m_driveSubsystem.teleopPID(-1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.gt(-Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.lt(0.0)), 
                                        ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.gt(-Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.lt(0.0)), 
                                        ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(10)
  @DisplayName("Test if robot can toggle traction control")
  public void toggleTractionControl() {
    // Hardcode NAVX sensor values for angle, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Enable traction control
    m_driveSubsystem.enableTractionControl();

    // Toggle off traction control
    m_driveSubsystem.toggleTractionControl();

    // Try to drive forward
    m_driveSubsystem.teleopPID(1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));

    // Try to drive in reverse
    m_driveSubsystem.teleopPID(-1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(-1.0, DELTA), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(-1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));

    // Toggle on traction control
    m_driveSubsystem.toggleTractionControl();

    // Try to drive forward
    m_driveSubsystem.teleopPID(1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.lt(Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.gt(0.0)), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.lt(Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.gt(0.0)), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));

    // Try to drive in reverse
    m_driveSubsystem.teleopPID(-1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.gt(-Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.lt(0.0)), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.and(AdditionalMatchers.gt(-Constants.DRIVE_SLIP_RATIO), AdditionalMatchers.lt(0.0)), 
                                         ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }
}
