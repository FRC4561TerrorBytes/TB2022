// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonPIDConfig;


public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private WPI_TalonFX flywheelMasterMotor, flywheelSlaveMotor, flywheelSmallMotor;
    private CANSparkMax upperFeederMotor, lowerFeederMotor;
    private SparkMaxLimitSwitch upperFeederSensor, lowerFeederSensor;
    private Counter lidar;

    public Hardware(WPI_TalonFX flywheelMasterMotor, 
                    WPI_TalonFX flywheelSlaveMotor,
                    WPI_TalonFX flywheelSmallMotor, 
                    CANSparkMax upperFeederMotor,
                    CANSparkMax lowerFeederMotor,
                    SparkMaxLimitSwitch upperFeederSensor,
                    SparkMaxLimitSwitch lowerFeederSensor,
                    Counter lidar) {
      this.flywheelMasterMotor = flywheelMasterMotor;
      this.flywheelSlaveMotor = flywheelSlaveMotor;
      this.flywheelSmallMotor = flywheelSmallMotor;
      this.upperFeederMotor = upperFeederMotor;
      this.lowerFeederMotor = lowerFeederMotor;
      this.upperFeederSensor = upperFeederSensor;
      this.lowerFeederSensor = lowerFeederSensor;
      this.lidar = lidar;
    }
  }

  public enum SelectedGoal {
    Low(0),
    High(1);

    int value;
    private SelectedGoal(int value) {
      this.value = value;
    }
  }

  @SuppressWarnings("unused")
  private static class BigFlywheel {
    private static final double MAX_SPEED_RPM = Constants.FALCON_500_MAX_RPM;
    private static final int TICKS_PER_ROTATION = Constants.CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
    private static WPI_TalonFX masterMotor;
    private static WPI_TalonFX slaveMotor;
    private static TalonPIDConfig masterConfig;
  }

  @SuppressWarnings("unused")
  private static class SmallFlywheel {
    private static final double MAX_SPEED_RPM = Constants.FALCON_500_MAX_RPM;
    private static final int TICKS_PER_ROTATION = Constants.CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
    private static WPI_TalonFX motor;
    private static TalonPIDConfig config;
  }

  private final double LIDAR_OFFSET = 9.0;

  private CANSparkMax m_upperFeederMotor;
  private CANSparkMax m_lowerFeederMotor;
  private SparkMaxLimitSwitch m_upperFeederSensor;
  private SparkMaxLimitSwitch m_lowerFeederSensor;
  private Counter m_lidar;

  private LinearFilter m_LIDARFilter;
  private PolynomialSplineFunction[] m_shooterOutputCurves = new PolynomialSplineFunction[2];
  private double[] m_maxDistance = new double[2];
  private SelectedGoal m_selectedGoal;

  private double m_smallFlywheelAddition;
  private double m_distance;
  private double m_feederIntakeSpeed;
  private double m_feederShootSpeed;

  /**
   * Creates an instance of ShooterSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!!!!!!
   * <p>
   * @param shooterHardware Hardware devices for shooter
   * @param flywheelMasterConfig PID config for flywheel
   * @param flywheelSmallConfig PID config for small flywheel
   * @param smallFlywheelAddition Delta between main flywheel and small flywheel speeds
   * @param feederIntakeSpeed Feeder intake speed in % [0.0, +1.0]
   * @param feederShootSpeed Feeder shoot speed in % [0.0, +1.0]
   * @param lowerShooterCurve Curve relating distance to flywheel speed for low goal
   * @param upperShooterCurve Curve relating distance to flywheel speed for high goal
   */
  public ShooterSubsystem(Hardware shooterHardware, TalonPIDConfig flywheelMasterConfig,
                          TalonPIDConfig flywheelSmallConfig, double smallFlywheelAddition, double feederIntakeSpeed, double feederShootSpeed, 
                          PolynomialSplineFunction lowerShooterCurve, PolynomialSplineFunction upperShooterCurve) {
    BigFlywheel.masterMotor = shooterHardware.flywheelMasterMotor;
    BigFlywheel.slaveMotor = shooterHardware.flywheelSlaveMotor;
    SmallFlywheel.motor = shooterHardware.flywheelSmallMotor;
    this.m_upperFeederMotor = shooterHardware.upperFeederMotor;
    this.m_lowerFeederMotor = shooterHardware.lowerFeederMotor;
    this.m_upperFeederSensor = shooterHardware.upperFeederSensor;
    this.m_lowerFeederSensor = shooterHardware.lowerFeederSensor;
    this.m_lidar = shooterHardware.lidar;
    this.m_smallFlywheelAddition = smallFlywheelAddition;
    this.m_shooterOutputCurves[0] = lowerShooterCurve;
    this.m_shooterOutputCurves[1] = upperShooterCurve;
    this.m_maxDistance[0] = lowerShooterCurve.getKnots()[lowerShooterCurve.getKnots().length - 1];
    this.m_maxDistance[1] = upperShooterCurve.getKnots()[upperShooterCurve.getKnots().length - 1];
    this.m_selectedGoal = SelectedGoal.Low;
    this.m_feederIntakeSpeed = feederIntakeSpeed;
    this.m_feederShootSpeed = feederShootSpeed;

    BigFlywheel.masterConfig = flywheelMasterConfig;
    SmallFlywheel.config = flywheelSmallConfig;

    // Reset feeder motors to default
    m_upperFeederMotor.restoreFactoryDefaults();
    m_lowerFeederMotor.restoreFactoryDefaults();

    // Set feeder motors to brake mode
    m_upperFeederMotor.setIdleMode(IdleMode.kBrake);
    m_lowerFeederMotor.setIdleMode(IdleMode.kBrake);

    // Set feeder motor inversion
    m_upperFeederMotor.setInverted(false);
    m_lowerFeederMotor.setInverted(true);

    // Enable beam beak sensor for only top feeder motor
    m_upperFeederSensor.enableLimitSwitch(true);
    m_lowerFeederSensor.enableLimitSwitch(false);

    // Initialize config for flywheel PID
    BigFlywheel.masterConfig.initializeTalonPID(BigFlywheel.masterMotor, FeedbackDevice.IntegratedSensor);
    BigFlywheel.slaveMotor.configFactoryDefault();
    BigFlywheel.slaveMotor.set(ControlMode.Follower, BigFlywheel.masterMotor.getDeviceID());
    BigFlywheel.slaveMotor.setInverted(InvertType.OpposeMaster);
    BigFlywheel.masterMotor.setNeutralMode(NeutralMode.Coast);
    BigFlywheel.slaveMotor.setNeutralMode(NeutralMode.Coast);

    // Initialize config for small flywheel PID
    SmallFlywheel.config.initializeTalonPID(SmallFlywheel.motor, FeedbackDevice.IntegratedSensor);
    SmallFlywheel.motor.setNeutralMode(NeutralMode.Coast);

    // Configure LIDAR settings
    m_LIDARFilter = LinearFilter.singlePoleIIR(0.4, Constants.ROBOT_LOOP_PERIOD);
    m_lidar.setMaxPeriod(1.0); // Set the max period that can be measured
    m_lidar.setSemiPeriodMode(true); // Set the counter to period measurement
    m_lidar.reset();
  }
  
  /**
   * Initialize hardware devices for shooter subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax upperFeederMotor = new CANSparkMax(Constants.UPPER_FEEDER_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax lowerFeederMotor = new CANSparkMax(Constants.LOWER_FEEDER_MOTOR_PORT, MotorType.kBrushless);
    Hardware shooterHardware = new Hardware(new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.FLYWHEEL_SMALL_MOTOR_PORT),
                                            upperFeederMotor,
                                            lowerFeederMotor,
                                            upperFeederMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed),
                                            lowerFeederMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed),
                                            new Counter(Constants.LIDAR_PORT));

    return shooterHardware;
  }

  /**
   * Create shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter Subsystem");
    tab.addNumber("Flywheel Motor Velocity (RPM)", () -> BigFlywheel.masterConfig.ticksPer100msToRPM(BigFlywheel.masterMotor.getSelectedSensorVelocity()));
    tab.addNumber("Flywheel Motor Setpoint (RPM)", () -> BigFlywheel.masterConfig.ticksPer100msToRPM(BigFlywheel.masterMotor.getClosedLoopTarget()));
    tab.addNumber("Flywheel Error (RPM)", () -> BigFlywheel.masterConfig.ticksPer100msToRPM(BigFlywheel.masterMotor.getClosedLoopError()));
    tab.addNumber("Flywheel Small Motor Velocity (RPM)", () -> SmallFlywheel.config.ticksPer100msToRPM(SmallFlywheel.motor.getSelectedSensorVelocity()));
    tab.addNumber("Flywheel Small Motor Setpoint (RPM)", () -> SmallFlywheel.config.ticksPer100msToRPM(SmallFlywheel.motor.getClosedLoopTarget()));
    tab.addNumber("Flywheel Small Error (RPM)", () -> SmallFlywheel.config.ticksPer100msToRPM(SmallFlywheel.motor.getClosedLoopError()));
    tab.addNumber("Shooter Distance (m)", () -> getDistance());
  }

  /**
   * Create SmartDashboard indicators
   */
  public void smartDashboard() {
    SmartDashboard.putBoolean("Ball 1", m_upperFeederSensor.isPressed());
    SmartDashboard.putBoolean("Ball 2", m_lowerFeederSensor.isPressed() && m_upperFeederSensor.isPressed());
    SmartDashboard.putBoolean("Selected Goal", m_selectedGoal == SelectedGoal.High);
  }

  @Override
  public void periodic() {
    smartDashboard();
    updateDistance();
  }

  /**
   * Select which goal to shoot for
   */
  public void selectGoal(SelectedGoal goal) {
    m_selectedGoal = goal;
  }

  /**
   * Toggles selected goal 
   */
  public void toggleSelectedGoal() {
    if (m_selectedGoal == SelectedGoal.Low) m_selectedGoal = SelectedGoal.High;
    else m_selectedGoal = SelectedGoal.Low;
  }

  /**
   * Automatically sets the flywheel speed based on distance from the goal.
   * @param distance Distance in meters
   */
  public void setFlywheelAuto() {
    //double distance = MathUtil.clamp(getDistance(), 0.0, m_maxDistance[m_selectedGoal.value]);
    setFlywheelSpeed(m_shooterOutputCurves[m_selectedGoal.value].value(0.0));
  }

  /**
   * Moves flywheel to a speed
   * @param speed input speed to keep the motor at (RPM)
   */
  public void setFlywheelSpeed(double speed) {
    double mainFlywheelSpeed = MathUtil.clamp(speed, 0, BigFlywheel.MAX_SPEED_RPM);
    double smallFlywheelSpeed = MathUtil.clamp(mainFlywheelSpeed * m_smallFlywheelAddition, 0, SmallFlywheel.MAX_SPEED_RPM);

    BigFlywheel.masterMotor.set(ControlMode.Velocity, BigFlywheel.masterConfig.rpmToTicksPer100ms(mainFlywheelSpeed));
    SmallFlywheel.motor.set(ControlMode.Velocity, SmallFlywheel.config.rpmToTicksPer100ms(smallFlywheelSpeed));
  }

  /**
   * Move flywheel at specified speed
   * @param speed flywheel speed [-1, +1]
   */
  public void flywheelManual(double speed) {
    BigFlywheel.masterMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stop flywheel motor
   */
  public void flywheelStop() {
    BigFlywheel.masterMotor.stopMotor();
    BigFlywheel.masterMotor.setIntegralAccumulator(0);
    SmallFlywheel.motor.stopMotor();
    SmallFlywheel.motor.setIntegralAccumulator(0);
  }

  /**
   * Checks if flywheel is at set speed
   * @return True if flywheel is at speed else false
   */
  public boolean isFlywheelAtSpeed() {
    double flywheelError = Math.abs(BigFlywheel.masterMotor.getClosedLoopError());
    double smallFlywheelError = Math.abs(SmallFlywheel.motor.getClosedLoopError());

    boolean isBigFlywheelAtSpeed = (flywheelError < BigFlywheel.masterConfig.getTolerance())
                                    && BigFlywheel.masterMotor.getClosedLoopTarget() != 0;
    boolean isSmallFlywheelAtSpeed = (smallFlywheelError < SmallFlywheel.config.getTolerance())
                                     && SmallFlywheel.motor.getClosedLoopTarget() != 0;
    
    // Ignore small flywheel speed for low
    isSmallFlywheelAtSpeed |= m_selectedGoal == SelectedGoal.Low;

    return isBigFlywheelAtSpeed && isSmallFlywheelAtSpeed;
  }

  /**
   * Returns whether or not the feeder is full
   */
  public boolean isFeederFull() {
    return m_upperFeederSensor.isPressed() && m_lowerFeederSensor.isPressed();
  }

  /**
   * Intake balls into feeder
   */
  public void feederIntake() {
    m_upperFeederSensor.enableLimitSwitch(true);
    m_upperFeederMotor.set(+m_feederIntakeSpeed);
    if (m_lowerFeederSensor.isPressed() && m_upperFeederSensor.isPressed()) {
      m_lowerFeederMotor.setOpenLoopRampRate(1.0);
      m_lowerFeederMotor.stopMotor();
    } else {
      m_lowerFeederMotor.setOpenLoopRampRate(0.0);
      m_lowerFeederMotor.set(+m_feederIntakeSpeed);
    }
  }

  /**
   * Outtake balls out of feeder
   */
  public void feederOuttake() {
    m_upperFeederSensor.enableLimitSwitch(false);
    m_lowerFeederMotor.setOpenLoopRampRate(0.0);
    m_upperFeederMotor.set(-m_feederIntakeSpeed);
    m_lowerFeederMotor.set(-m_feederIntakeSpeed);
  }

  /**
   * Shoot balls out of feeder
   */
  public void feederShoot() {
    m_upperFeederSensor.enableLimitSwitch(false);
    m_lowerFeederMotor.setOpenLoopRampRate(0.0);
    m_upperFeederMotor.set(+m_feederShootSpeed);
    m_lowerFeederMotor.set(+m_feederShootSpeed);
  }

  /**
   * Stops feeder motors
   */
  public void feederStop() {
    m_lowerFeederMotor.setOpenLoopRampRate(1.0);
    m_upperFeederMotor.stopMotor();
    m_lowerFeederMotor.stopMotor();
  }

  /**
   * Update and filter distance readings from LIDAR
   */
  public void updateDistance() {
    double lidarOutput = 0;
    double lidarPeriod = m_LIDARFilter.calculate(m_lidar.getPeriod());
    if (m_lidar.get() < 1) {
      m_distance = lidarOutput;
    } else {
      lidarOutput = ((lidarPeriod * 1000000.0 / 10.0) - LIDAR_OFFSET) / 100.0; // convert to distance. sensor is high 10 us for every centimeter.
      m_distance = Math.copySign(Math.floor(Math.abs(lidarOutput) * 100) / 100, lidarOutput);
    }
  }

  /**
   * Get latest filtered distance reading from LIDAR
   * @return Distance in meters
   */
  public double getDistance() {
    return m_distance;
  }

  @Override
  public void close() {
    BigFlywheel.masterMotor = null;
    BigFlywheel.slaveMotor = null;
    SmallFlywheel.motor = null;
    m_upperFeederMotor = null;
    m_lowerFeederMotor = null;
    m_upperFeederSensor = null;
    m_lowerFeederSensor = null;
  }
}
