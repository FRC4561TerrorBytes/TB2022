// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  // Motor RPMs, encoder values, and gear ratios
  public static final int FALCON_500_MAX_RPM = 6380;
  public static final int BAG_MAX_RPM = 13180;
  public static final int CTRE_MAG_ENCODER_TICKS_PER_ROTATION = 4096;
  public static final int CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION = 2048;

  public static final double DRIVE_WHEEL_DIAMETER_METERS = 0.1524;
  public static final double DRIVE_GEAR_RATIO = 120.0 / 11.0;

  // Drive PID values
  public static final double DRIVE_kP = 0.018;
  public static final double DRIVE_kD = 0.00029;
  public static final double DRIVE_TURN_SCALAR = 30.0;
  public static final double DRIVE_ACCELERATION_LIMIT = 0.5;
  public static final String DRIVE_TRACTION_CONTROL_CURVE = "X / 4.106";
  public static final String DRIVE_THROTTLE_INPUT_CURVE = "4.106 * X";
  public static final int DRIVE_RESPONSE_EXPONENT = 1;

  // Climber PID variables
  public static final double CLIMBER_kP = 0.0;
  public static final double CLIMBER_kD = 0.0;
  public static final double CLIMBER_TOLERANCE = 10;
  public static final double CLIMBER_LOWER_LIMIT = -1500;
  public static final double CLIMBER_UPPER_LIMIT = 0;
  public static final double CLIMBER_VELOCITY = FALCON_500_MAX_RPM;
  public static final double CLIMBER_ACCLERATION = FALCON_500_MAX_RPM;
  public static final int CLIMBER_MOTION_SMOOTHING = 1;
  public static final int CLIMBER_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
  public static final int CLIMBER_MAX_RPM = FALCON_500_MAX_RPM;
  public static final boolean CLIMBER_SOFT_LIMITS = true;
  public static final boolean CLIMBER_SENSOR_PHASE = false;
  public static final boolean CLIMBER_INVERT_MOTOR = false;

  // Climber PID config
  public static final TalonPIDConfig CLIMBER_CONFIG = new TalonPIDConfig(CLIMBER_SENSOR_PHASE, 
                                                                         CLIMBER_INVERT_MOTOR, 
                                                                         CLIMBER_TICKS_PER_ROTATION, 
                                                                         CLIMBER_MAX_RPM, 
                                                                         CLIMBER_kP, 
                                                                         0.0, 
                                                                         CLIMBER_kD, 
                                                                         CLIMBER_TOLERANCE, 
                                                                         CLIMBER_LOWER_LIMIT, 
                                                                         CLIMBER_UPPER_LIMIT, 
                                                                         CLIMBER_SOFT_LIMITS, 
                                                                         CLIMBER_VELOCITY, 
                                                                         CLIMBER_ACCLERATION, 
                                                                         CLIMBER_MOTION_SMOOTHING);


  // Xbox controller ports
  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;

  // Drive Motor Ports 
  public static final int FRONT_LEFT_MOTOR_PORT = 0;
  public static final int REAR_LEFT_MOTOR_PORT = 1;

  public static final int FRONT_RIGHT_MOTOR_PORT = 2;
  public static final int REAR_RIGHT_MOTOR_PORT = 3;

  // Intake Motor Ports
  public static final int ARM_MOTOR_PORT = 0;
  public static final int INTAKE_ROLLER_PORT = 1;

  // Intake Arm Positions
  public static final int ARM_TOP_POSITION = -1450;
  public static final int ARM_BOTTOM_POSITION = 0;
  // Climber motor ports
  public static final int CLIMBER_MOTOR_PORT = 42;
}
