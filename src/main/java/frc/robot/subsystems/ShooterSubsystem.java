// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonPIDConfig;


public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private WPI_TalonFX flywheelMasterMotor, flywheelSlaveMotor;
    private WPI_TalonSRX upperFeederMotor;
    private WPI_TalonSRX lowerFeederMotor;

    private Counter lidar;

    public Hardware(WPI_TalonFX flywheelMasterMotor, 
                    WPI_TalonFX flywheelSlaveMotor, 
                    WPI_TalonSRX upperFeederMotor,
                    WPI_TalonSRX lowerFeederMotor,
                    Counter lidar) {
      this.flywheelMasterMotor = flywheelMasterMotor;
      this.flywheelSlaveMotor = flywheelSlaveMotor;
      this.upperFeederMotor = upperFeederMotor;
      this.lowerFeederMotor = lowerFeederMotor;
      this.lidar = lidar;
    }
  }

  @SuppressWarnings("unused")
  private static class Flywheel {
    private static final double MAX_SPEED_RPM = Constants.FALCON_500_MAX_RPM;
    private static final int TICKS_PER_ROTATION = Constants.CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
    private static WPI_TalonFX masterMotor;
    private static WPI_TalonFX slaveMotor;
    private static TalonPIDConfig masterConfig;
  }

  private WPI_TalonSRX m_upperFeederMotor;
  private WPI_TalonSRX m_lowerFeederMotor;
  private Counter m_lidar;
  private final double LIDAR_OFFSET = 10.0;
  /**
   * Creates an instance of ShooterSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!!!!!!
   * <p>
   * @param shooterHardware Hardware devices for shooter
   * @param flywheelMasterConfig PID config for flywheel
   */
  public ShooterSubsystem(Hardware shooterHardware, TalonPIDConfig flywheelMasterConfig) {
    Flywheel.masterMotor = shooterHardware.flywheelMasterMotor;
    Flywheel.slaveMotor = shooterHardware.flywheelSlaveMotor;
    m_upperFeederMotor = shooterHardware.upperFeederMotor;
    m_lowerFeederMotor = shooterHardware.lowerFeederMotor;
    m_lidar = shooterHardware.lidar;

    Flywheel.masterConfig = flywheelMasterConfig;

    m_upperFeederMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    m_lowerFeederMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Initialize config for flywheel PID
    Flywheel.masterConfig.initializeTalonPID(Flywheel.masterMotor, TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), false, false);
    Flywheel.slaveMotor.set(ControlMode.Follower, Flywheel.masterMotor.getDeviceID());
    Flywheel.slaveMotor.setInverted(true);

    // Configure LIDAR settings
    m_lidar.setMaxPeriod(1.00); //set the max period that can be measured
    m_lidar.setSemiPeriodMode(true); //Set the counter to period measurement
    m_lidar.reset();
  }
  
  /**
   * Initialize hardware devices for shooter subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT),
                                            new WPI_TalonSRX(Constants.FEEDER_MOTOR_PORT),
                                            new WPI_TalonSRX(Constants.FEEDER_MOTOR_PORT),
                                            new Counter(Constants.LIDAR_PORT));
    return shooterHardware;
  }

  /**
   * Create shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter Subsystem");
    Shuffleboard.getTab("SmartDashboard").addBoolean("Flywheel at Speed?", () -> isFlywheelAtSpeed());
    tab.addNumber("Flywheel Motor Output", () -> Flywheel.masterMotor.getMotorOutputPercent());
    tab.addNumber("Flywheel Current", () -> Flywheel.masterMotor.getSupplyCurrent());
    tab.addNumber("Flywheel Motor Velocity", () -> Flywheel.masterConfig.ticksPer100msToRPM(Flywheel.masterMotor.getSelectedSensorVelocity()));
    tab.addNumber("Flywheel Motor Setpoint", () -> Flywheel.masterConfig.ticksPer100msToRPM(Flywheel.masterMotor.getClosedLoopTarget()));
    tab.addNumber("Flywheel Error", () -> Flywheel.masterMotor.getClosedLoopError());
    tab.addNumber("Shooter Distance", () -> getLIDAR());
  }

  @Override
  public void periodic() {}

  /**
   * Moves flywheel to a speed
   * @param speed input speed to keep the motor at (RPM)
   */
  public void setFlywheelSpeed(double speed) {
    speed = MathUtil.clamp(speed, 0, Flywheel.MAX_SPEED_RPM);
    double speedInTicks = Flywheel.masterConfig.rpmToTicksPer100ms(speed);

    Flywheel.masterMotor.set(ControlMode.Velocity, speedInTicks);
  }

  /**
   * Move flywheel at specified speed
   * @param speed flywheel speed [-1, +1]
   */
  public void flywheelManual(double speed) {
    Flywheel.masterMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stop flywheel motor
   */
  public void flywheelStop() {
    Flywheel.masterMotor.stopMotor();
    Flywheel.masterMotor.setIntegralAccumulator(0);
  }

  /**
   * Checks if flywheel is at set speed
   * @return True if flywheel is at speed else false
   */
  public boolean isFlywheelAtSpeed() {
    return (Math.abs(Flywheel.masterMotor.getClosedLoopError()) < Flywheel.masterConfig.getTolerance())
                      && Flywheel.masterMotor.getClosedLoopTarget() != 0;
  }

  /**
   * Sets feeder intake speed
   */
  public void feederIntake() {
    m_upperFeederMotor.set(ControlMode.PercentOutput, Constants.FEEDER_INTAKE_SPEED);
    m_lowerFeederMotor.set(ControlMode.PercentOutput, Constants.FEEDER_INTAKE_SPEED);
  }

  /**
   * Sets feeder outtake speed
   */
  public void feederOuttake() {
    m_upperFeederMotor.overrideLimitSwitchesEnable(false);
    m_lowerFeederMotor.overrideLimitSwitchesEnable(false);
    m_upperFeederMotor.set(ControlMode.PercentOutput, -Constants.FEEDER_INTAKE_SPEED);
    m_lowerFeederMotor.set(ControlMode.PercentOutput, -Constants.FEEDER_INTAKE_SPEED);
  }

  /**
   * Sets feeder shoot speed
   */
  public void feederShoot() {
    m_upperFeederMotor.overrideLimitSwitchesEnable(false);
    m_lowerFeederMotor.overrideLimitSwitchesEnable(false);
    m_upperFeederMotor.set(ControlMode.PercentOutput, Constants.FEEDER_SHOOT_SPEED);
    m_lowerFeederMotor.set(ControlMode.PercentOutput, Constants.FEEDER_SHOOT_SPEED);
  }

  /**
   * Stops feeder motor
   */
  public void feederStop() {
    m_upperFeederMotor.overrideLimitSwitchesEnable(true);
    m_lowerFeederMotor.overrideLimitSwitchesEnable(true);
    m_upperFeederMotor.stopMotor();
    m_lowerFeederMotor.stopMotor();
  }

  /**
   * Get Distance from LIDAR sensor
   * @return distance in Meters
   */
  public double getLIDAR() {
    if(m_lidar.get() < 1)
      return 0;
    else
      return ((m_lidar.getPeriod()*1000000.0/10.0) - LIDAR_OFFSET) / 100.0; //convert to distance. sensor is high 10 us for every centimeter. 
  }

  @Override
  public void close() {
    Flywheel.masterMotor = null;
    Flywheel.slaveMotor = null;
    m_upperFeederMotor = null;
    m_lowerFeederMotor = null;
  }
}
