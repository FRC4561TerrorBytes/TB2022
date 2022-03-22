// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import frc.robot.utils.TalonPIDConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Robot tick rate in seconds
  public static final double ROBOT_LOOP_PERIOD = 1.0 / 60.0;

  // Automode Constants
  public static final double TRACK_WIDTH = 0.71452;

  // Controller deadband
  public static final double CONTROLLER_DEADBAND = 0.12;

  // Spline interpolator
  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();

  // Motor RPMs, encoder values, and gear ratios
  public static final int FALCON_500_MAX_RPM = 6380;
  public static final int BAG_MAX_RPM = 13180;
  public static final int CTRE_MAG_ENCODER_TICKS_PER_ROTATION = 4096;
  public static final int CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION = 2048;

  // Drive specs
  public static final double DRIVE_WHEEL_DIAMETER_METERS = 0.1016;
  public static final double DRIVE_GEAR_RATIO = 1680.0 / 220.0;
  public static final double DRIVE_TICKS_PER_METER = (CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION * DRIVE_GEAR_RATIO) * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI)); // 48997.324
  public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER; // 2.041e-5
  public static final double DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION; // 0.041798
  public static final double DRIVETRAIN_EFFICIENCY = 0.85;
  public static final double DRIVE_MAX_LINEAR_SPEED = (FALCON_500_MAX_RPM / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY; // 3.766 m/s

  // Drive PID values
  public static final double DRIVE_kP = 0.008;
  public static final double DRIVE_kD = 0.00012;
  public static final double DRIVE_TURN_SCALAR = 75.0;
  public static final double DRIVE_LOOKAHEAD = 6;

  private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.5,   1.0 };
  private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 1.883, 3.766 };
  private static final double DRIVE_TRACTION_CONTROL_CURVE_X[] = { 0.0, 1.883, 3.766 };
  private static final double DRIVE_TRACTION_CONTROL_CURVE_Y[] = { 0.0, 0.5,   1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.5, 1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.5, 1.0 };

  public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
  public static final PolynomialSplineFunction DRIVE_TRACTION_CONTROL_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TRACTION_CONTROL_CURVE_X, DRIVE_TRACTION_CONTROL_CURVE_Y);
  public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

  private static final double CURRENT_LIMIT = 60.0;
  private static final double CURRENT_THRESHOLD = 120.0;
  private static final double CURRENT_THRESHOLD_TIME = 6 * ROBOT_LOOP_PERIOD;
  
  public static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT_CONFIGURATION = new StatorCurrentLimitConfiguration(true, CURRENT_LIMIT, CURRENT_THRESHOLD, CURRENT_THRESHOLD_TIME);

  // Intake Arm PID config
  public static final double INTAKE_ARM_kP = 0.8;
  public static final double INTAKE_ARM_kD = 0.0;
  public static final double INTAKE_ARM_MECHANICAL_EFFICIENCY = 0.8;
  public static final double INTAKE_ARM_TOLERANCE = 10;
  public static final double INTAKE_ARM_LOWER_LIMIT = 0;
  public static final double INTAKE_ARM_UPPER_LIMIT = 3100;
  public static final double INTAKE_ARM_VELOCITY = FALCON_500_MAX_RPM;
  public static final double INTAKE_ARM_ACCELERATION = FALCON_500_MAX_RPM * 12;
  public static final int INTAKE_ARM_MOTION_SMOOTHING = 6;
  public static final int INTAKE_ARM_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  public static final int INTAKE_ARM_MAX_RPM = FALCON_500_MAX_RPM;
  public static final boolean INTAKE_ARM_SOFT_LIMITS = true;
  public static final boolean INTAKE_ARM_SENSOR_PHASE = false;
  public static final boolean INTAKE_ARM_INVERT_MOTOR = false;

  public static final double INTAKE_ROLLER_SPEED = 1.0;

  // Intake Arm PID config
  public static final TalonPIDConfig INTAKE_ARM_CONFIG = new TalonPIDConfig(INTAKE_ARM_SENSOR_PHASE, 
                                                                            INTAKE_ARM_INVERT_MOTOR, 
                                                                            INTAKE_ARM_TICKS_PER_ROTATION, 
                                                                            INTAKE_ARM_MAX_RPM, 
                                                                            INTAKE_ARM_kP, 
                                                                            0.0, 
                                                                            INTAKE_ARM_kD, 
                                                                            INTAKE_ARM_MECHANICAL_EFFICIENCY,
                                                                            INTAKE_ARM_TOLERANCE, 
                                                                            INTAKE_ARM_LOWER_LIMIT, 
                                                                            INTAKE_ARM_UPPER_LIMIT, 
                                                                            INTAKE_ARM_SOFT_LIMITS, 
                                                                            INTAKE_ARM_VELOCITY, 
                                                                            INTAKE_ARM_ACCELERATION, 
                                                                            INTAKE_ARM_MOTION_SMOOTHING);

  // Shooter PID Values
  private static final double FLYWHEEL_kP = 0.002;
  private static final double FLYWHEEL_kI = 0.0;
  private static final double FLYWHEEL_kD = 0.00025;
  private static final double FLYWHEEL_MECHANICAL_EFFICIENCY = 1.0;
  private static final double FLYWHEEL_TOLERANCE = 80;
  private static final double FLYWHEEL_MAX_RPM = FALCON_500_MAX_RPM;
  private static final double FLYWHEEL_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private static final boolean FLYWHEEL_MASTER_ENCODER_SENSOR_PHASE = false;
  private static final boolean FLYWHEEL_MASTER_MOTOR_INVERTED = true;
  public static final double FLYWHEEL_SHOOTING_RPM = 1700;
  public static final double FEEDER_INTAKE_SPEED = 0.2;
  public static final double FEEDER_SHOOT_SPEED = 0.3;

  private static final double SHOOTER_LOW_CURVE_X[] = { 0.0, 0.5, 1.0 };
  private static final double SHOOTER_LOW_CURVE_Y[] = { 1620.0, 1620.0, 1620.0 };
  private static final double SHOOTER_HIGH_CURVE_X[] = { 0.0, 0.5, 1.0 };
  private static final double SHOOTER_HIGH_CURVE_Y[] = { 3200.0, 3200.0, 3200.0 };

  public static final PolynomialSplineFunction SHOOTER_LOW_CURVE = SPLINE_INTERPOLATOR.interpolate(SHOOTER_LOW_CURVE_X, SHOOTER_LOW_CURVE_Y);
  public static final PolynomialSplineFunction SHOOTER_HIGH_CURVE = SPLINE_INTERPOLATOR.interpolate(SHOOTER_HIGH_CURVE_X, SHOOTER_HIGH_CURVE_Y);


  // Set PID for Flywheel
  public static final TalonPIDConfig FLYWHEEL_MASTER_CONFIG = new TalonPIDConfig(FLYWHEEL_MASTER_ENCODER_SENSOR_PHASE,
                                                                                 FLYWHEEL_MASTER_MOTOR_INVERTED,
                                                                                 FLYWHEEL_MAX_RPM,
                                                                                 FLYWHEEL_TICKS_PER_ROTATION,
                                                                                 FLYWHEEL_kP,
                                                                                 FLYWHEEL_kI,
                                                                                 FLYWHEEL_kD,
                                                                                 FLYWHEEL_MECHANICAL_EFFICIENCY,
                                                                                 FLYWHEEL_TOLERANCE);

  private static final double FLYWHEEL_SMALL_kP = 0.004;
  private static final double FLYWHEEL_SMALL_kI = 1e-5;
  private static final double FLYWHEEL_SMALL_kD = 0.0;
  private static final double FLYWHEEL_SMALL_MECHANICAL_EFFICIENCY = 1.04;
  private static final double FLYWHEEL_SMALL_TOLERANCE = 80;
  private static final double FLYWHEEL_SMALL_MAX_RPM = FALCON_500_MAX_RPM;
  private static final double FLYWHEEL_SMALL_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private static final boolean FLYWHEEL_SMALL_ENCODER_SENSOR_PHASE = false;
  private static final boolean FLYWHEEL_SMALL_MOTOR_INVERTED = true;
  public static final double FLYWHEEL_SMALL_RPM_HIGH = 2560;
  public static final double FLYWHEEL_SMALL_RPM_LOW = 162;

  // Set PID for Flywheel small
  public static final TalonPIDConfig FLYWHEEL_SMALL_CONFIG = new TalonPIDConfig(FLYWHEEL_SMALL_ENCODER_SENSOR_PHASE,
                                                                                FLYWHEEL_SMALL_MOTOR_INVERTED,
                                                                                FLYWHEEL_SMALL_MAX_RPM,
                                                                                FLYWHEEL_SMALL_TICKS_PER_ROTATION,
                                                                                FLYWHEEL_SMALL_kP,
                                                                                FLYWHEEL_SMALL_kI,
                                                                                FLYWHEEL_SMALL_kD,
                                                                                FLYWHEEL_SMALL_MECHANICAL_EFFICIENCY,
                                                                                FLYWHEEL_SMALL_TOLERANCE);

  // Telescope PID variables
  public static final double TELESCOPE_kP = 0.1;
  public static final double TELESCOPE_kD = 0.0;
  public static final double TELESCOPE_MECHANICAL_EFFICIENCY = 0.9;
  public static final double TELESCOPE_TOLERANCE = 100;
  public static final double TELESCOPE_LOWER_LIMIT = 0;
  public static final double TELESCOPE_UPPER_LIMIT = 290000;
  public static final double TELESCOPE_VELOCITY = FALCON_500_MAX_RPM;
  public static final double TELESCOPE_ACCELERATION = FALCON_500_MAX_RPM;
  public static final int TELESCOPE_MOTION_SMOOTHING = 4;
  public static final int TELESCOPE_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  public static final int TELESCOPE_MAX_RPM = FALCON_500_MAX_RPM;
  public static final boolean TELESCOPE_SOFT_LIMITS = true;
  public static final boolean TELESCOPE_SENSOR_PHASE = false;
  public static final boolean TELESCOPE_INVERT_MOTOR = true;

  // Telescope PID config
  public static final TalonPIDConfig TELESCOPE_CONFIG = new TalonPIDConfig(TELESCOPE_SENSOR_PHASE, 
                                                                           TELESCOPE_INVERT_MOTOR, 
                                                                           TELESCOPE_TICKS_PER_ROTATION, 
                                                                           TELESCOPE_MAX_RPM, 
                                                                           TELESCOPE_kP, 
                                                                           0.0, 
                                                                           TELESCOPE_kD, 
                                                                           TELESCOPE_MECHANICAL_EFFICIENCY,
                                                                           TELESCOPE_TOLERANCE, 
                                                                           TELESCOPE_LOWER_LIMIT, 
                                                                           TELESCOPE_UPPER_LIMIT, 
                                                                           TELESCOPE_SOFT_LIMITS, 
                                                                           TELESCOPE_VELOCITY, 
                                                                           TELESCOPE_ACCELERATION, 
                                                                           TELESCOPE_MOTION_SMOOTHING);

  // Winch PID variables
  public static final double WINCH_kP = 0.1;
  public static final double WINCH_kD = 0.0;
  public static final double WINCH_MECHANICAL_EFFICIENCY = 0.8;
  public static final double WINCH_TOLERANCE = 1000;
  public static final double WINCH_LOWER_LIMIT = 0;
  public static final double WINCH_UPPER_LIMIT = 265000;
  public static final double WINCH_VELOCITY = FALCON_500_MAX_RPM * 0.5;
  public static final double WINCH_ACCELERATION = FALCON_500_MAX_RPM * 0.5;
  public static final int WINCH_MOTION_SMOOTHING = 4;
  public static final int WINCH_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  public static final int WINCH_MAX_RPM = FALCON_500_MAX_RPM;
  public static final boolean WINCH_SOFT_LIMITS = true;
  public static final boolean WINCH_SENSOR_PHASE = false;
  public static final boolean WINCH_INVERT_MOTOR = false;

  // Winch PID config
  public static final TalonPIDConfig WINCH_CONFIG = new TalonPIDConfig(WINCH_SENSOR_PHASE, 
                                                                       WINCH_INVERT_MOTOR, 
                                                                       WINCH_TICKS_PER_ROTATION, 
                                                                       WINCH_MAX_RPM, 
                                                                       WINCH_kP, 
                                                                       0.0, 
                                                                       WINCH_kD, 
                                                                       WINCH_MECHANICAL_EFFICIENCY,
                                                                       WINCH_TOLERANCE, 
                                                                       WINCH_LOWER_LIMIT, 
                                                                       WINCH_UPPER_LIMIT, 
                                                                       WINCH_SOFT_LIMITS, 
                                                                       WINCH_VELOCITY, 
                                                                       WINCH_ACCELERATION, 
                                                                       WINCH_MOTION_SMOOTHING);

  // Xbox controller ports
  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;

  // PDH port
  public static final int PDH_PORT = 50;

  // Drive hardware Ports 
  public static final int FRONT_LEFT_MOTOR_PORT = 1;
  public static final int REAR_LEFT_MOTOR_PORT = 2;

  public static final int FRONT_RIGHT_MOTOR_PORT = 3;
  public static final int REAR_RIGHT_MOTOR_PORT = 4;

  // Intake hardware Ports
  public static final int ARM_MOTOR_PORT = 5;
  public static final int INTAKE_ROLLER_PORT = 6;

  // Shooter hardware ports
  public static final int FLYWHEEL_MASTER_MOTOR_PORT = 7;
  public static final int FLYWHEEL_SLAVE_MOTOR_PORT = 8;
  public static final int UPPER_FEEDER_MOTOR_PORT = 9;
  public static final int LOWER_FEEDER_MOTOR_PORT = 10;
  public static final int FLYWHEEL_SMALL_MOTOR_PORT = 15;
  public static final int LIDAR_PORT = 0;

  // Climber hardware ports
  public static final int CLIMBER_MOTOR_PORT = 11;
  public static final int CLIMBER_WINCH_MOTOR_PORT = 12;
  public static final int CLIMBER_LEFT_TELESCOPE_MOTOR_PORT = 13;
  public static final int CLIMBER_RIGHT_TELESCOPE_MOTOR_PORT = 14;

  // Accessories
  public static final int BLINKIN_LED_CONTROLLER_PORT = 0;
}
