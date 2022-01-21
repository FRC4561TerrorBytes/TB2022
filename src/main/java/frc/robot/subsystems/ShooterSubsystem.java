// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;


public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private WPI_TalonFX flywheelMasterMotor, flywheelSlaveMotor;
    private WPI_TalonSRX hoodMotor;

    public Hardware(WPI_TalonFX flywheelMasterMotor, WPI_TalonFX flywheelSlaveMotor, WPI_TalonSRX hoodMotor) {
      this.flywheelMasterMotor = flywheelMasterMotor;
      this.flywheelSlaveMotor = flywheelSlaveMotor;
      this.hoodMotor = hoodMotor;
    }
  }

  private static class Flywheel {
    private static final double MAX_SPEED_RPM = Constants.FALCON_500_MAX_RPM;
    private static final int TICKS_PER_ROTATION = Constants.CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
    private static WPI_TalonFX masterMotor;
    private static WPI_TalonFX slaveMotor;
    private static TalonPIDConfig masterConfig;
    private static boolean isRunning = false;

    private static double rpmToTicksPer100ms(double speed) {
      return (speed * TICKS_PER_ROTATION) / 600;
    }

    private static double ticksToRPM(double speed) {
      return (speed * 600) / TICKS_PER_ROTATION;
    }
  }

  private static class Hood {
    private static boolean needsReset = true;
    private static int topPosition = Constants.HOOD_TOP_POSITION;
    private static double bottomPosition = Constants.HOOD_BOTTOM_POSITION;
    private static WPI_TalonSRX motor;
    private static TalonPIDConfig config;
  }
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Hardware shooterHardware, TalonPIDConfig flywheelMasterConfig, TalonPIDConfig hoodConfig) {
    Flywheel.masterMotor = shooterHardware.flywheelMasterMotor;
    Flywheel.slaveMotor = shooterHardware.flywheelSlaveMotor;
    Hood.motor = shooterHardware.hoodMotor;

    Flywheel.masterConfig = flywheelMasterConfig;
    Hood.config = hoodConfig;

    // Initialize config for flywheel PID
    Flywheel.masterConfig.initializeTalonPID(Flywheel.masterMotor, TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), false, false);
    Flywheel.slaveMotor.set(ControlMode.Follower, Flywheel.masterMotor.getDeviceID());
    Flywheel.slaveMotor.setInverted(true);

    // Initialize config for hood and turret PID
    Hood.config.initializeTalonPID(Hood.motor, FeedbackDevice.CTRE_MagEncoder_Relative, true, false);

    // Reset encoders to 0 on initialisation
    resetEncoder(Flywheel.masterMotor);
    resetEncoder(Hood.motor);
  }

  /**
   * Initialize hardware devices for shooter subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT),
                                            new WPI_TalonSRX(Constants.HOOD_MOTOR_PORT));
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
    tab.addNumber("Flywheel Motor Velocity", () -> Flywheel.ticksToRPM(Flywheel.masterMotor.getSelectedSensorVelocity()));
    tab.addNumber("Flywheel Motor Setpoint", () -> Flywheel.ticksToRPM(Flywheel.masterMotor.getClosedLoopTarget()));
    tab.addNumber("Flywheel Error", () -> flywheelError());

    tab.addNumber("Hood Motor Output", () -> Hood.motor.getMotorOutputPercent());
    tab.addNumber("Hood Motor Current", () -> Hood.motor.getSupplyCurrent());
    tab.addNumber("Hood Encoder Position", () -> Hood.motor.getSelectedSensorPosition());
    tab.addNumber("Hood Error", () -> Hood.motor.getClosedLoopError());
  }
  
  /**
   * Reset the Turret and Hood setpoints
   */
  public void reset() {
    moveHoodPID(Hood.bottomPosition);
  }

  /**
   * Moves hood to position
   * @param setpoint input position to move to (in ticks)
   */
  public void moveHoodPID(double setpoint) {
    // Normalise setpoint
    if (setpoint < Hood.topPosition) setpoint = Hood.topPosition;
    else if (setpoint > Hood.bottomPosition) setpoint = Hood.bottomPosition;

    // Move hood toward setpoint
    Hood.motor.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Toggles hood between top and bottom positions
   */
  public void toggleHoodPosition() {
    if (Hood.motor.getClosedLoopTarget() == Constants.HOOD_TOP_POSITION) {
      moveHoodPID(Constants.HOOD_BOTTOM_POSITION);
    } else if (Hood.motor.getClosedLoopTarget() == Constants.HOOD_BOTTOM_POSITION) {
      moveHoodPID(Constants.HOOD_TOP_POSITION);
    }
  }

  /**
   * Move hood at specified speed
   * @param speed speed to move hood at [-1, 1]
   */
  public void hoodManual(double speed) {
    Hood.motor.set(speed);
  }

  /**
   * Moves flywheel to a speed
   * @param speed input speed to keep the motor at (RPM)
   */
  public void setFlywheelSpeed(double speed) {
    speed = MathUtil.clamp(speed, 0, Flywheel.MAX_SPEED_RPM);
    double speedInTicks = Flywheel.rpmToTicksPer100ms(speed);

    // PID controller on Talon uses 1023 as "full output" 
    Flywheel.masterMotor.set(ControlMode.Velocity, speedInTicks);
  }

  /**
   * Move flywheel at specified speed
   * @param speed flywheel speed [-1, 1]
   */
  public void flywheelManual(double speed) {
    Flywheel.masterMotor.set(speed);
  }

  /**
   * Stop flywheel motor
   */
  public void flywheelStop() {
    Flywheel.masterMotor.set(0);
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
   * Reset encoder to 0 to keep it in sync
   * @param motor resets input encoder
   */
  public void resetEncoder(BaseTalon motor) {
    motor.setSelectedSensorPosition(0);
  }

  public void toggleFlywheel(double speed) {
    if (!Flywheel.isRunning) {
      flywheelManual(speed);
      Flywheel.isRunning = true;
    }
    else {
      flywheelStop();
      Flywheel.isRunning = false;
    }
  }

  /**
   * @return if the back limit switch is pressed
   */
  public boolean hoodLimit() {
    return Hood.motor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    // Reset the hood encoder to the back position
    if (hoodLimit() && Hood.needsReset) {
      Hood.needsReset = false;
      Hood.motor.setSelectedSensorPosition(Constants.HOOD_BOTTOM_POSITION);
    } else if (!hoodLimit()) Hood.needsReset = true;
  }

  @Override
  public void close() {
    Flywheel.masterMotor = null;
    Flywheel.slaveMotor = null;
    Hood.motor = null;
  }
}
