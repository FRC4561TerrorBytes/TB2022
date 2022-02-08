// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonPIDConfig;


public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private WPI_TalonFX flywheelMasterMotor, flywheelSlaveMotor;
    private CANSparkMax feederMotor;
    private SparkMaxLimitSwitch forwardLimitSwitch;

    public Hardware(WPI_TalonFX flywheelMasterMotor, WPI_TalonFX flywheelSlaveMotor, CANSparkMax feederMotor) {
      this.flywheelMasterMotor = flywheelMasterMotor;
      this.flywheelSlaveMotor = flywheelSlaveMotor;
      this.feederMotor = feederMotor;
      this.forwardLimitSwitch = feederMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

      forwardLimitSwitch.enableLimitSwitch(true);
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

  private CANSparkMax m_feederMotor;
  private SparkMaxLimitSwitch m_feederForwardLimitSwitch;

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
    m_feederMotor = shooterHardware.feederMotor;

    Flywheel.masterConfig = flywheelMasterConfig;

    // Initialize config for flywheel PID
    Flywheel.masterConfig.initializeTalonPID(Flywheel.masterMotor, TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), false, false);
    Flywheel.slaveMotor.set(ControlMode.Follower, Flywheel.masterMotor.getDeviceID());
    Flywheel.slaveMotor.setInverted(true);
  }
  
  /**
   * Initialize hardware devices for shooter subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT),
                                            new CANSparkMax(Constants.FEEDER_MOTOR_PORT, MotorType.kBrushless));
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
    tab.addNumber("Flywheel Error", () -> flywheelError());
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
    Flywheel.masterMotor.set(speed);
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
    return (Math.abs(flywheelError()) < Flywheel.masterConfig.getTolerance())
                      && Flywheel.masterMotor.getClosedLoopTarget() != 0;
  }

  /**
   * Get error in flywheel speed
   * @return flywheel error (ticks per 100ms)
   */
  public double flywheelError() {
    return Flywheel.masterMotor.getClosedLoopTarget() - Flywheel.masterMotor.getSelectedSensorVelocity();
  }

  /**
   * Sets feeder intake speed
   */
  public void feederIntake() {
    m_feederMotor.set(Constants.FEEDER_INTAKE_SPEED);
  }

  /**
   * Sets feeder outtake speed
   */
  public void feederOuttake() {
    m_feederMotor.set(-Constants.FEEDER_INTAKE_SPEED);
  }

  /**
   * Sets feeder shoot speed
   */
  public void feederShoot() {
    m_feederForwardLimitSwitch.enableLimitSwitch(false);
    m_feederMotor.set(Constants.FEEDER_SHOOT_SPEED);
  }

  /**
   * Stops feeder motor
   */
  public void feederStop() {
    m_feederForwardLimitSwitch.enableLimitSwitch(true);
    m_feederMotor.stopMotor();
  }

  @Override
  public void close() {
    Flywheel.masterMotor = null;
    Flywheel.slaveMotor = null;
    m_feederMotor = null;
  }
}
