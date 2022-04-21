// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import frc.robot.subsystems.ShooterSubsystem.FlywheelSpeed;
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
  public static final double TRACK_WIDTH = 0.82124;

  // Controller deadband
  public static final double CONTROLLER_DEADBAND = 0.12;

  // Spline interpolator
  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();

  // Motor RPMs, encoder values, and gear ratios
  public static final int FALCON_500_MAX_RPM = 6380;
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
  public static final double DRIVE_kP = 0.02;
  public static final double DRIVE_kD = 0.0004;
  public static final double DRIVE_TURN_SCALAR = 18.0;
  public static final double DRIVE_LOOKAHEAD = 16;

  private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.5,   1.0 };
  private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 1.883, 3.766 };
  private static final double DRIVE_TRACTION_CONTROL_CURVE_X[] = { 0.0, 1.883, 3.766 };
  private static final double DRIVE_TRACTION_CONTROL_CURVE_Y[] = { 0.0, 0.5,   1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.01, 0.04, 0.09, 0.16, 0.25, 0.36, 0.49, 0.64, 0.81, 1.0 };

  public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
  public static final PolynomialSplineFunction DRIVE_TRACTION_CONTROL_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TRACTION_CONTROL_CURVE_X, DRIVE_TRACTION_CONTROL_CURVE_Y);
  public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

  private static final double CURRENT_LIMIT = 100.0;
  private static final double CURRENT_THRESHOLD = 200.0;
  private static final double CURRENT_THRESHOLD_TIME = 6 * ROBOT_LOOP_PERIOD;
  
  public static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT_CONFIGURATION = new StatorCurrentLimitConfiguration(true, CURRENT_LIMIT, CURRENT_THRESHOLD, CURRENT_THRESHOLD_TIME);

  // Intake Arm PID config
  private static final double INTAKE_ARM_kP = 0.8;
  private static final double INTAKE_ARM_kD = 0.0;
  private static final double INTAKE_ARM_MECHANICAL_EFFICIENCY = 0.8;
  private static final double INTAKE_ARM_TOLERANCE = 10;
  private static final double INTAKE_ARM_LOWER_LIMIT = 0;
  private static final double INTAKE_ARM_UPPER_LIMIT = 3100;
  private static final double INTAKE_ARM_VELOCITY = FALCON_500_MAX_RPM;
  private static final double INTAKE_ARM_ACCELERATION = FALCON_500_MAX_RPM * 12;
  private static final int INTAKE_ARM_MOTION_SMOOTHING = 6;
  private static final int INTAKE_ARM_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private static final int INTAKE_ARM_MAX_RPM = FALCON_500_MAX_RPM;
  private static final boolean INTAKE_ARM_SOFT_LIMITS = true;
  private static final boolean INTAKE_ARM_SENSOR_PHASE = false;
  private static final boolean INTAKE_ARM_INVERT_MOTOR = false;

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
  private static final double FLYWHEEL_kP = 0.011;
  private static final double FLYWHEEL_kI = 0.0;
  private static final double FLYWHEEL_kD = 0.001;
  private static final double FLYWHEEL_MECHANICAL_EFFICIENCY = 1.01;
  private static final double FLYWHEEL_TOLERANCE = 80;
  private static final double FLYWHEEL_MAX_RPM = FALCON_500_MAX_RPM;
  private static final double FLYWHEEL_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private static final boolean FLYWHEEL_MASTER_ENCODER_SENSOR_PHASE = false;
  private static final boolean FLYWHEEL_MASTER_MOTOR_INVERTED = true;

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

  private static final double FLYWHEEL_SMALL_kP = 0.18;
  private static final double FLYWHEEL_SMALL_kI = 0.0008;
  private static final double FLYWHEEL_SMALL_kD = 0.012;
  private static final double FLYWHEEL_SMALL_MECHANICAL_EFFICIENCY = 1.0;
  private static final double FLYWHEEL_SMALL_TOLERANCE = 200;
  private static final double FLYWHEEL_SMALL_MAX_RPM = FALCON_500_MAX_RPM;
  private static final double FLYWHEEL_SMALL_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private static final boolean FLYWHEEL_SMALL_ENCODER_SENSOR_PHASE = false;
  private static final boolean FLYWHEEL_SMALL_MOTOR_INVERTED = true;

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

  // Shooter settings
  public static final double FEEDER_INTAKE_SPEED = 0.2;
  public static final double FEEDER_SHOOT_SPEED = 0.3;
  public static final double SHOOT_DELAY = 0.2;
  public static final double FLYWHEEL_SMALL_VISION_IDLE_RAMP = 1;
  public static final FlywheelSpeed SPIT_OUT_LOW_FLYWHEEL_SPEED = new FlywheelSpeed(1200.0, 100.0);
  public static final FlywheelSpeed SPIT_OUT_HIGH_FLYWHEEL_SPEED = new FlywheelSpeed(5000.0, 5000.0);
  public static final FlywheelSpeed LOW_FLYWHEEL_SPEED = new FlywheelSpeed(1620.0, 162.0);
  public static final FlywheelSpeed HIGH_FLYWHEEL_SPEED = new FlywheelSpeed(2700.0, 700.0);
  public static final FlywheelSpeed FLYWHEEL_IDLE_SPEED = new FlywheelSpeed(500.0, 500.0);
  public static final boolean FLYWHEEL_IDLE_DEFAULT_ENABLED = false;
  public static final List<Entry<Double, FlywheelSpeed>> FLYWHEEL_VISION_MAP = Arrays.asList(
    Map.entry(0.00, new FlywheelSpeed(2300.0, 900.0)),
    Map.entry(1.10, new FlywheelSpeed(2300.0, 900.0)),
    Map.entry(1.20, new FlywheelSpeed(2300.0, 900.0)),
    Map.entry(1.30, new FlywheelSpeed(2350.0, 940.0)),
    Map.entry(1.40, new FlywheelSpeed(2330.0, 955.0)),
    Map.entry(1.50, new FlywheelSpeed(2250.0, 1150.0)),
    Map.entry(1.60, new FlywheelSpeed(2200.0, 1200.0)),
    Map.entry(1.70, new FlywheelSpeed(2150.0, 1400.0)),
    Map.entry(1.80, new FlywheelSpeed(2050.0, 1600.0)),
    Map.entry(2.00, new FlywheelSpeed(1800.0, 2100.0)),
    Map.entry(2.25, new FlywheelSpeed(1700.0, 2200.0)),
    Map.entry(2.50, new FlywheelSpeed(1750.0, 2500.0)),
    Map.entry(2.75, new FlywheelSpeed(1600.0, 3000.0)),
    Map.entry(3.00, new FlywheelSpeed(1730.0, 3300.0)),
    Map.entry(3.05, new FlywheelSpeed(1750.0, 3150.0)),
    Map.entry(3.15, new FlywheelSpeed(1600.0, 3500.0)),
    Map.entry(3.25, new FlywheelSpeed(1575.0, 3675.0)),
    Map.entry(3.50, new FlywheelSpeed(1550.0, 3800.0)),
    Map.entry(3.75, new FlywheelSpeed(1500.0, 3900.0)),
    Map.entry(4.00, new FlywheelSpeed(1500.0, 3950.0))
  );

  // Telescope PID variables
  private static final double TELESCOPE_kP = 0.1;
  private static final double TELESCOPE_kI = 0.0;
  private static final double TELESCOPE_kD = 0.0;
  private static final double TELESCOPE_MECHANICAL_EFFICIENCY = 0.8;
  private static final double TELESCOPE_TOLERANCE = 100;
  private static final double TELESCOPE_LOWER_LIMIT = -290000;
  private static final double TELESCOPE_UPPER_LIMIT = 0;
  private static final double TELESCOPE_VELOCITY = FALCON_500_MAX_RPM * 0.125;
  private static final double TELESCOPE_ACCELERATION = FALCON_500_MAX_RPM * 0.125;
  private static final int TELESCOPE_MOTION_SMOOTHING = 2;
  private static final int TELESCOPE_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private static final int TELESCOPE_MAX_RPM = FALCON_500_MAX_RPM;
  private static final boolean TELESCOPE_SOFT_LIMITS = true;
  private static final boolean TELESCOPE_SENSOR_PHASE = false;
  private static final boolean TELESCOPE_INVERT_MOTOR = false;

  // Telescope PID config
  public static final TalonPIDConfig TELESCOPE_CONFIG = new TalonPIDConfig(TELESCOPE_SENSOR_PHASE, 
                                                                           TELESCOPE_INVERT_MOTOR, 
                                                                           TELESCOPE_TICKS_PER_ROTATION, 
                                                                           TELESCOPE_MAX_RPM, 
                                                                           TELESCOPE_kP, 
                                                                           TELESCOPE_kI, 
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
  private static final double WINCH_kP = 0.1;
  private static final double WINCH_kD = 0.0;
  private static final double WINCH_MECHANICAL_EFFICIENCY = 0.8;
  private static final double WINCH_TOLERANCE = 4000;
  private static final double WINCH_LOWER_LIMIT = 0;
  private static final double WINCH_UPPER_LIMIT = 330000;
  private static final double WINCH_VELOCITY = FALCON_500_MAX_RPM * 0.6;
  private static final double WINCH_ACCELERATION = FALCON_500_MAX_RPM * 0.6;
  private static final int WINCH_MOTION_SMOOTHING = 2;
  private static final int WINCH_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  private static final int WINCH_MAX_RPM = FALCON_500_MAX_RPM;
  private static final boolean WINCH_SOFT_LIMITS = true;
  private static final boolean WINCH_SENSOR_PHASE = false;
  private static final boolean WINCH_INVERT_MOTOR = false;

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

  // Vision constants
  public static final double CAMERA_HEIGHT_METERS = 0.792;
  public static final double TARGET_HEIGHT_METERS = 2.642;
  public static final double CAMERA_PITCH_DEGREES = 41.8;
  public static final double VISION_MAX_DISTANCE = FLYWHEEL_VISION_MAP.get(FLYWHEEL_VISION_MAP.size() - 1).getKey();


  // Xbox controller ports
  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;

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

  // Climber hardware ports
  public static final int CLIMBER_WINCH_MOTOR_PORT = 12;
  public static final int CLIMBER_LEFT_TELESCOPE_MOTOR_PORT = 13;
  public static final int CLIMBER_RIGHT_TELESCOPE_MOTOR_PORT = 14;

  // Accessories
  public static final int BLINKIN_LED_CONTROLLER_PORT = 0;

  // SmartDashboard keys
  public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
  public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  public static final String SMARTDASHBOARD_FLYWHEEL_IDLE_ENABLED = "Flywheel Idle";
  public static final String SMARTDASHBOARD_FLYWHEEL_BIG_INPUT = "Big";
  public static final String SMARTDASHBOARD_FLYWHWEEL_SMALL_INPUT = "Small";
}
