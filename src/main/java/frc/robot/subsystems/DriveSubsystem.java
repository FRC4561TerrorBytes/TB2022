/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.StringWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TractionControlController;
import frc.robot.utils.TurnPIDController;


public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private WPI_TalonFX lMasterMotor, rMasterMotor;
    private WPI_TalonFX lSlaveMotor, rSlaveMotor;
    private AHRS navx;

    public Hardware(WPI_TalonFX lMasterMotor, 
                    WPI_TalonFX rMasterMotor, 
                    WPI_TalonFX lSlaveMotor,
                    WPI_TalonFX rSlaveMotor,
                    AHRS navx) {
      this.lMasterMotor = lMasterMotor;
      this.rMasterMotor = rMasterMotor;
      this.lSlaveMotor = lSlaveMotor;
      this.rSlaveMotor = rSlaveMotor;
      this.navx = navx;
    }
  }

  private String SUBSYSTEM_NAME = "Drive Subsystem";

  private TurnPIDController m_turnPIDController;
  private TractionControlController m_tractionControlController;
  private DifferentialDriveOdometry m_odometry;

  private WPI_TalonFX m_lMasterMotor;
  private WPI_TalonFX m_lSlaveMotor;

  private WPI_TalonFX m_rMasterMotor;
  private WPI_TalonFX m_rSlaveMotor;

  private AHRS m_navx;

  private final double TOLERANCE = 0.125;
  private final double MOTOR_DEADBAND = 0.005;
  private final double MAX_VOLTAGE = 12.0;

  private double m_turnScalar = 1.0; 
  private double m_metersPerTick = 0.0;
  private double m_deadband = 0.0;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param turnScalar Scalar for turn input (degrees)
   * @param deadband Deadband for controller input [+0.001, +0.1]
   * @param lookAhead Turn PID lookahead, in number of loops
   * @param metersPerTick Meters traveled per encoder tick (meters)
   * @param maxLinearSpeed Maximum linear speed of the robot (m/s)
   * @param tractionControlCurve Spline function characterising traction of the robot
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   * @param currentLimitConfiguration Drive current limit
   */
  public DriveSubsystem(Hardware drivetrainHardware, double kP, double kD, double turnScalar, double deadband, double lookAhead, double metersPerTick, double maxLinearSpeed, 
                        PolynomialSplineFunction tractionControlCurve, PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve,
                        StatorCurrentLimitConfiguration currentLimitConfiguration) {
    m_turnPIDController = new TurnPIDController(kP, kD, turnScalar, deadband, lookAhead, turnInputCurve);
    m_tractionControlController = new TractionControlController(deadband, maxLinearSpeed, tractionControlCurve, throttleInputCurve);

    this.m_lMasterMotor = drivetrainHardware.lMasterMotor;
    this.m_rMasterMotor = drivetrainHardware.rMasterMotor;
    this.m_lSlaveMotor = drivetrainHardware.lSlaveMotor;
    this.m_rSlaveMotor = drivetrainHardware.rSlaveMotor;

    this.m_navx = drivetrainHardware.navx;

    this.m_deadband = deadband;
    this.m_turnScalar = turnScalar;
    this.m_metersPerTick = metersPerTick;

    // Reset TalonFX settings
    m_lMasterMotor.configFactoryDefault();
    m_lSlaveMotor.configFactoryDefault();
    m_rMasterMotor.configFactoryDefault();
    m_rSlaveMotor.configFactoryDefault();

    // Set all drive motors to brake
    m_lMasterMotor.setNeutralMode(NeutralMode.Brake);
    m_lSlaveMotor.setNeutralMode(NeutralMode.Brake);
    m_rMasterMotor.setNeutralMode(NeutralMode.Brake);
    m_rSlaveMotor.setNeutralMode(NeutralMode.Brake);

    // Make rear left motor controllers follow left master
    m_lSlaveMotor.set(ControlMode.Follower, m_lMasterMotor.getDeviceID());

    // Make rear right motor controllers follow right master
    m_rSlaveMotor.set(ControlMode.Follower, m_rMasterMotor.getDeviceID());

    // Make motors use integrated encoder
    m_lMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Reduce motor deadband
    m_lMasterMotor.configNeutralDeadband(MOTOR_DEADBAND);
    m_lSlaveMotor.configNeutralDeadband(MOTOR_DEADBAND);
    m_rMasterMotor.configNeutralDeadband(MOTOR_DEADBAND);
    m_rSlaveMotor.configNeutralDeadband(MOTOR_DEADBAND);

    // Enable voltage compensation
    m_lMasterMotor.configVoltageCompSaturation(MAX_VOLTAGE);
    m_lMasterMotor.enableVoltageCompensation(true);
    m_lSlaveMotor.configVoltageCompSaturation(MAX_VOLTAGE);
    m_lSlaveMotor.enableVoltageCompensation(true);
    m_rMasterMotor.configVoltageCompSaturation(MAX_VOLTAGE);
    m_rMasterMotor.enableVoltageCompensation(true);
    m_rSlaveMotor.configVoltageCompSaturation(MAX_VOLTAGE);
    m_rSlaveMotor.enableVoltageCompensation(true);

    // Enable current limits
    m_lMasterMotor.configStatorCurrentLimit(currentLimitConfiguration);
    m_lSlaveMotor.configStatorCurrentLimit(currentLimitConfiguration);
    m_rMasterMotor.configStatorCurrentLimit(currentLimitConfiguration);
    m_rSlaveMotor.configStatorCurrentLimit(currentLimitConfiguration);

    // Initialise PID subsystem setpoint and input
    m_navx.calibrate();
    resetAngle();
    m_turnPIDController.setSetpoint(0.0);

    // Set drive PID tolerance
    m_turnPIDController.setTolerance(TOLERANCE);

    // Initialise odometry
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware drivetrainHardware = new Hardware(new WPI_TalonFX(Constants.FRONT_LEFT_MOTOR_PORT),
                                               new WPI_TalonFX(Constants.FRONT_RIGHT_MOTOR_PORT),
                                               new WPI_TalonFX(Constants.REAR_LEFT_MOTOR_PORT),
                                               new WPI_TalonFX(Constants.REAR_RIGHT_MOTOR_PORT),
                                               new AHRS(SPI.Port.kMXP));

    return drivetrainHardware;
  }

  /**
   * Initialize drive subsystem for autonomous
   */
  public void autonomousInit() {
     // Invert only left side
     m_lMasterMotor.setInverted(true);
     m_lSlaveMotor.setInverted(true);
     m_rMasterMotor.setInverted(false);
     m_rSlaveMotor.setInverted(false);
  }

  /**
   * Initialize drive subsystem for teleop
   */
  public void teleopInit() {
    resetDrivePID();

    // Invert only right side
    m_lMasterMotor.setInverted(false);
    m_lSlaveMotor.setInverted(false);
    m_rMasterMotor.setInverted(true);
    m_rSlaveMotor.setInverted(true);

    System.out.println("teleopInit@!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  /**
   * Create Shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Drive Angle (degrees)", () -> getAngle());
    tab.addNumber("Right master current (amps)", () -> m_rMasterMotor.getSupplyCurrent());
    tab.addNumber("Left master current (amps)", () -> m_lMasterMotor.getSupplyCurrent());
    tab.addNumber("Right slave current (amps)", () -> m_rSlaveMotor.getSupplyCurrent());
    tab.addNumber("Left slave current (amps)", () -> m_lSlaveMotor.getSupplyCurrent());
  }

  @Override
  public void periodic() {}

  /**
   * Call this repeatedly to drive without PID during teleoperation
   * @param speed Desired speed [-1.0, +1.0]
   * @param turn Turn input [-1.0, +1.0]
   * @param power exponent for drive response curve. 1 is linear response
   */
	public void teleop(double speed, double turn, int power) {
    speed = Math.copySign(Math.pow(speed, power), speed);
    turn = Math.copySign(Math.pow(turn, power), turn);

    speed = MathUtil.applyDeadband(speed, m_deadband);
    turn = MathUtil.applyDeadband(turn, m_deadband);

    m_lMasterMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, -turn);
    m_rMasterMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, +turn);
	}

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param speedRequest Desired speed [-1.0, +1.0]
   * @param turnRequest Turn input [-1.0, +1.0]
   */
  public void teleopPID(double speedRequest, double turnRequest) {
    // Calculate next PID turn output
    double turnOutput = m_turnPIDController.calculate(getAngle(), getTurnRate(), turnRequest);

    // Calculate next speed output
    double speedOutput = m_tractionControlController.calculate(getInertialVelocity(), speedRequest);

    // Run motors with appropriate values
    m_lMasterMotor.set(ControlMode.PercentOutput, speedOutput, DemandType.ArbitraryFeedForward, -turnOutput);
    m_rMasterMotor.set(ControlMode.PercentOutput, speedOutput, DemandType.ArbitraryFeedForward, +turnOutput);
  }

  /**
   * Maintain robot angle using PID
   */
  public void maintainAngle() {
    double turnOutput = m_turnPIDController.calculate(getAngle());

    m_lMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, -turnOutput);
    m_rMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, +turnOutput);
  }

  /**
   * Turn robot by angleDelta
   * @param angleDelta degrees to turn robot by [-turnScalar, +turnScalar]
   */
  public void aimToAngle(double angleDelta) throws InterruptedException {
    angleDelta = MathUtil.clamp(angleDelta, -m_turnScalar, +m_turnScalar);
    m_turnPIDController.setSetpoint(getAngle() + angleDelta);
    long loopTime = (long)Constants.ROBOT_LOOP_PERIOD * 1000;

    do {
      double turnOutput = m_turnPIDController.calculate(getAngle());
      m_lMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, -turnOutput);
      m_rMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, +turnOutput);
      
      Thread.sleep(loopTime);
    } while (!m_turnPIDController.atSetpoint());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * <p>
   * Only use this method to drive during autonomous!
   * @param leftVolts Left voltage [-12, +12]
   * @param rightVolts Right voltage [-12, +12]
   */
  public void autoTankDriveVolts(double leftVolts, double rightVolts) {
    m_lMasterMotor.setVoltage(leftVolts);
    m_rMasterMotor.setVoltage(rightVolts);
    System.out.println("driving!!!");
  }

  /**
   * Set speed of left and right drive seperately
   * @param leftSpeed speed [-1, 1]
   * @param rightSpeed speed [-1, 1]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_lMasterMotor.set(ControlMode.PercentOutput, leftSpeed, DemandType.ArbitraryFeedForward, 0.0);
    m_rMasterMotor.set(ControlMode.PercentOutput, rightSpeed, DemandType.ArbitraryFeedForward, 0.0);
  }
  
  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_lMasterMotor.getSelectedSensorVelocity() * 10 * m_metersPerTick, 
                                            m_rMasterMotor.getSelectedSensorVelocity() * 10 * m_metersPerTick);
  }

  /**
   * Resets the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetAngle();
    resetEncoders();
    m_odometry.resetPosition(pose, m_navx.getRotation2d());
  }

  /**
   * Update robot odometry
   * <p>
   * Repeatedly call this method at a steady rate to keep track of robot position
   */
  public void updateOdometry() {
    m_odometry.update(m_navx.getRotation2d(), 
                      m_lMasterMotor.getSelectedSensorPosition() * m_metersPerTick,
                      m_rMasterMotor.getSelectedSensorPosition() * m_metersPerTick);
  }

  /**
   * Returns the currently estimated pose of the robot
   * <p>
   * This method is called periodically by the Ramsete command to update and obtain the latest pose
   * @return The pose
   */
  public Pose2d getPose() {
    updateOdometry();
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return m_navx.getYaw();
  }
  
  /**
   * Zeros the heading of the robot
   */
  public void zeroHeading() {
    resetAngle();
  }

  /**
   * Reset left and right drive
   */
  public void resetEncoders() {
    m_lMasterMotor.setSelectedSensorPosition(0.0);
    m_rMasterMotor.setSelectedSensorPosition(0.0);
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate();
  }

  /**
   * Returns inertial velocity of the robot.
   * @return Velocity of the robot as measured by the NAVX
   */
  public double getInertialVelocity() {
    return m_navx.getVelocityY();
  }

  /**
   * Converts encoder position to meters
   * @param ticks Encoder position
   * @return Return distance in meters
   */
  public double getDistance(double ticks) {
    return ticks * m_metersPerTick;
  }

  /**
   * Gets the average distance of the two encoders.
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (((m_lMasterMotor.getSensorCollection().getIntegratedSensorPosition() * m_metersPerTick) + 
             (m_lMasterMotor.getSensorCollection().getIntegratedSensorPosition() * m_metersPerTick)) / 2);
  }

  /**
   * Stop drivetrain
   */
  public void stop() {
    m_lMasterMotor.disable();
    m_rMasterMotor.disable();
    System.out.println("stopping!!!");
    
    Thread.dumpStack();
  }

  /**
   * Get DriveSubsystem angle as detected by the navX MXP
   * @return Total accumulated yaw angle
   */
  public double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Reset Drivesubsystem navX MXP yaw angle to specific angle
   * @param angle angle to reset navX to
   */
  public void resetAngle() {
    m_navx.reset();
  }
  
  /**
   * Reset DriveSubsystem PID
   */
  public void resetDrivePID() {
    resetAngle();
    m_turnPIDController.setSetpoint(0.0);
    m_turnPIDController.reset();
  }

  /**
   * Get setpoint for drive PID
   * @return current setpoint in degrees
   */
  public double getDrivePIDSetpoint() {
    return m_turnPIDController.getSetpoint();
  }

  @Override
  public void close() {
    m_lMasterMotor = null;
    m_rMasterMotor = null;
    m_lSlaveMotor = null;
    m_rSlaveMotor = null;
    m_navx = null;
  }
}
