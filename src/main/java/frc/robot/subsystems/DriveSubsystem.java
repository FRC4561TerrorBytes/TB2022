/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TractionControlController;


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

  private PIDController m_drivePIDController;
  private TractionControlController m_tractionControlController;
  private DifferentialDriveOdometry m_odometry;

  private WPI_TalonFX m_lMasterMotor;
  private WPI_TalonFX m_lSlaveMotor;

  private WPI_TalonFX m_rMasterMotor;
  private WPI_TalonFX m_rSlaveMotor;

  private AHRS m_navx;

  private final double TURN_DEADBAND = 0.005;
  private final double WHEEL_DIAMETER_METERS = Constants.DRIVE_WHEEL_DIAMETER_METERS;
  private final double MOTOR_MAX_RPM = Constants.FALCON_500_MAX_RPM;
  private final double TICKS_PER_ROTATION = Constants.CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private final double GEAR_RATIO = Constants.DRIVE_GEAR_RATIO;
  private final double TICKS_PER_METER = (double)(TICKS_PER_ROTATION * GEAR_RATIO) * (double)(1 / (WHEEL_DIAMETER_METERS * Math.PI)); // 46644.183
  private final double METERS_PER_TICK = 1 / TICKS_PER_METER; // 2.149e-5
  private final double METERS_PER_ROTATION = METERS_PER_TICK * TICKS_PER_ROTATION; // 0.04388
  private final double DRIVETRAIN_EFFICIENCY = 0.88;
  private final double MAX_LINEAR_SPEED = Math.floor(((MOTOR_MAX_RPM / 60) * METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY) * 1000) / 1000; // 4.106 m/s
  private final double TOLERANCE = 0.125;

  private double m_turnScalar = 1.0; 
  private double m_inertialVelocity = 0.0;
  private boolean m_wasTurning = false;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param turnScalar Scalar for turn input (degrees)
   * @param accelerationLimit Maximum allowed acceleration (m/s^2)
   * @param tractionControlCurve Expression characterising traction of the robot with "X" as the variable
   * @param throttleInputCurve Expression characterising throttle input with "X" as the variable
   */
  public DriveSubsystem(Hardware drivetrainHardware, double kP, double kD, double turnScalar,
                        double accelerationLimit, String tractionControlCurve, String throttleInputCurve) {
      m_drivePIDController = new PIDController(kP, 0.0, kD, Constants.ROBOT_LOOP_PERIOD);
      m_tractionControlController = new TractionControlController(MAX_LINEAR_SPEED, accelerationLimit, tractionControlCurve, throttleInputCurve);

      this.m_lMasterMotor = drivetrainHardware.lMasterMotor;
      this.m_rMasterMotor = drivetrainHardware.rMasterMotor;
      this.m_lSlaveMotor = drivetrainHardware.lSlaveMotor;
      this.m_rSlaveMotor = drivetrainHardware.rSlaveMotor;

      this.m_navx = drivetrainHardware.navx;

      this.m_turnScalar = turnScalar;

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

      // Invert only right side
      m_lMasterMotor.setInverted(false);
      m_lSlaveMotor.setInverted(false);
      m_rMasterMotor.setInverted(true);
      m_rSlaveMotor.setInverted(true);

      // Make rear left motor controllers follow left master
      m_lSlaveMotor.set(ControlMode.Follower, m_lMasterMotor.getDeviceID());

      // Make rear right motor controllers follow right master
      m_rSlaveMotor.set(ControlMode.Follower, m_rMasterMotor.getDeviceID());

      // Make motors use integrated encoder
      m_lMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      m_rMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

      // Initialise PID subsystem setpoint and input
      resetAngle();
      m_drivePIDController.setSetpoint(0.0);

      // Set drive PID tolerance
      m_drivePIDController.setTolerance(TOLERANCE);

      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
      resetOdometry();
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
  public void periodic() {
    // Display traction control indicator on SmartDashboard
    SmartDashboard.putBoolean("TC", m_tractionControlController.isEnabled());
    
    // Update the odometry in the periodic block
    // Negate gyro angle because gyro is positive going clockwise which doesn't match WPILib convention
    m_odometry.update(Rotation2d.fromDegrees(-getAngle()), 
                      -m_lMasterMotor.getSelectedSensorPosition() * METERS_PER_TICK,
                      m_rMasterMotor.getSelectedSensorPosition() * METERS_PER_TICK);
  }

  /**
   * Call this repeatedly to drive without PID during teleoperation
   * @param speed Desired speed [-1.0, +1.0]
   * @param turn Turn input [-1.0, +1.0]
   * @param power exponent for drive response curve. 1 is linear response
   */
	public void teleop(double speed, double turn, int power) {
    speed = Math.copySign(Math.pow(speed, power), speed);
    turn = Math.copySign(Math.pow(turn, power), turn);

    m_lMasterMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, -turn);
    m_rMasterMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, +turn);
	}

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param speedRequest Desired speed [-1.0, +1.0]
   * @param turnRequest Turn input [-1.0, +1.0]
   */
  public void teleopPID(double speedRequest, double turnRequest) {
    double currentAngle = getAngle();

    // Start turning if input is greater than deadband
    if (Math.abs(turnRequest) >= TURN_DEADBAND) {
      // Add delta to setpoint scaled by factor
      m_drivePIDController.setSetpoint(currentAngle + (turnRequest * m_turnScalar));
      m_wasTurning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (m_wasTurning) {
        m_drivePIDController.setSetpoint(currentAngle);
        m_wasTurning = false;
      }
    }

    // Calculate next PID turn output
    double turnOutput = m_drivePIDController.calculate(currentAngle);

    // Calculate next motor speed output
    double optimalSpeedOutput = m_tractionControlController.calculate(getInertialVelocity(), speedRequest);

    // Run motors with appropriate values
    m_lMasterMotor.set(ControlMode.PercentOutput, optimalSpeedOutput, DemandType.ArbitraryFeedForward, -turnOutput);
    m_rMasterMotor.set(ControlMode.PercentOutput, optimalSpeedOutput, DemandType.ArbitraryFeedForward, +turnOutput);
  }

  /**
   * Turn robot by angleDelta
   * @param angleDelta degrees to turn robot by
   */
  public void aimToAngle(double angleDelta) {
    double currentAngle = getAngle();

    m_drivePIDController.setSetpoint(currentAngle + angleDelta);

    double turnOutput = m_drivePIDController.calculate(currentAngle);

    m_lMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, -turnOutput);
    m_rMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, +turnOutput);
  }

  /**
   * Turn robot to set angle
   * @param angleSetpoint In degrees [-180, +180]
   * @return True when complete
   */
  public boolean turnToAngle(double angleSetpoint) {
    MathUtil.clamp(angleSetpoint, -180.0, +180.0);
    resetAngle();
    m_drivePIDController.setSetpoint(angleSetpoint);

    double currentAngle = getAngle();
    while (Math.abs(currentAngle) <= Math.abs(angleSetpoint)) {
      double output = m_drivePIDController.calculate(currentAngle, m_drivePIDController.getSetpoint());
      m_lMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, -output);
      m_rMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, output);
      currentAngle = getAngle();
    }

    return true;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts  the commanded left output [-12, +12]
   * @param rightVolts  the commanded right output [-12, +12]
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_lMasterMotor.setVoltage(-leftVolts);
    m_rMasterMotor.setVoltage(+rightVolts);
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
    return new DifferentialDriveWheelSpeeds(m_lMasterMotor.getSelectedSensorVelocity() * 10 * METERS_PER_TICK, 
                                            m_rMasterMotor.getSelectedSensorVelocity() * 10 * METERS_PER_TICK);
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry() {
    resetAngle();
    m_lMasterMotor.setSelectedSensorPosition(0.0);
    m_rMasterMotor.setSelectedSensorPosition(0.0);
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0.0));
  }

  /**
   * Returns the currently-estimated pose of the robot
   * @return The pose
   */
  public Pose2d getPose() {
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
    return -m_navx.getRate();
  }

  /**
   * Returns inertial velocity of the robot.
   * @return Velocity of the robot as measured by the NAVX
   */
  public double getInertialVelocity() {
    // Return inertial velocity to nearest cm/sec
    m_inertialVelocity = m_navx.getVelocityY();
    m_inertialVelocity = Math.copySign(Math.floor(Math.abs(m_inertialVelocity) * 100) / 100, m_inertialVelocity);
    return m_inertialVelocity;
  }

  /**
   * Converts encoder position to meters
   * @param ticks Encoder position
   * @return Return distance in meters
   */
  public double getDistance(double ticks) {
    return ticks * METERS_PER_TICK;
  }

  /**
   * Gets the average distance of the two encoders.
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (((m_lMasterMotor.getSensorCollection().getIntegratedSensorPosition() * METERS_PER_TICK) + 
              (m_lMasterMotor.getSensorCollection().getIntegratedSensorPosition() * METERS_PER_TICK)) / 2);
  }

  /**
   * Stop drivetrain
   */
  public void stop() {
    m_lMasterMotor.set(ControlMode.PercentOutput, 0.0);
    m_rMasterMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_tractionControlController.toggle();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_tractionControlController.disable();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_tractionControlController.enable();
  }

  /**
   * Get DriveSubsystem angle as detected by the navX MXP
   * @return Total accumulated yaw angle
   */
  public double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Reset DriveSubsystem navX MXP yaw angle
   */
  public void resetAngle() {
    m_navx.reset();
  }
  
  /**
   * Reset DriveSubsystem PID
   */
  public void resetDrivePID() {
    resetAngle();
    m_drivePIDController.setSetpoint(0.0);
    m_drivePIDController.reset();
  }

  /**
   * Get setpoint for drive PID
   * @return current setpoint in degrees
   */
  public double getDrivePIDSetpoint() {
    return m_drivePIDController.getSetpoint();
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
