// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonPIDConfig;


public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private WPI_TalonFX armMotor;
    private CANSparkMax rollerMotor;

    public Hardware(WPI_TalonFX armMotor, 
                    CANSparkMax rollerMotor) {
      this.armMotor = armMotor;
      this.rollerMotor = rollerMotor;
    }
  }

  public enum ArmPosition {
    Top(Constants.INTAKE_ARM_UPPER_LIMIT),
    Bottom(Constants.INTAKE_ARM_LOWER_LIMIT);

    public final double value;
    private ArmPosition(double value) {
      this.value = value;
    }
  }

  private final String SUBSYSTEM_NAME = "Intake Subsystem";

  private WPI_TalonFX m_armMotor;
  private CANSparkMax m_rollerMotor;
  private TalonPIDConfig m_armConfig;
  private ArmPosition m_armPosition;
  private ArmPosition m_prevArmPosition;
  private double m_rollerSpeed;

  private final double ARM_MIDDLE_POSITION = ArmPosition.Bottom.value / 2;

  /**
   * Create an instance of IntakeSubsystem
   * <p>
   * ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param intakeHardware Hardware devices required by intake
   * @param armConfig PID config for arm
   * @param rollerSpeed Intake roller speed [-1.0, +1.0]
   */
  public IntakeSubsystem(Hardware intakeHardware, TalonPIDConfig armConfig, double rollerSpeed) {
    this.m_armMotor = intakeHardware.armMotor;
    this.m_rollerMotor = intakeHardware.rollerMotor;
    this.m_armConfig = armConfig;
    this.m_rollerSpeed = rollerSpeed;

    m_armPosition = ArmPosition.Top;
    m_prevArmPosition = m_armPosition;
    
    m_rollerMotor.setInverted(false);

    m_armConfig.initializeTalonPID(m_armMotor, FeedbackDevice.IntegratedSensor);
    m_armMotor.setSelectedSensorPosition(0.0);
  }

  /**
   * Initialize hardware devices for intake subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(new WPI_TalonFX(Constants.ARM_MOTOR_PORT),
                                           new CANSparkMax(Constants.INTAKE_ROLLER_PORT, MotorType.kBrushless));

    return intakeHardware;
  }

  /**
   * Create Shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Arm position (ticks)", () -> m_armMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets arm speed to percentage
   * @param speed Speed percentage [-1.0, +1.0]
   */
  public void armManual(double speed) {
    m_armMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Move arm to position
   * @param setpoint position to move arm to (ticks)
   */
  public void armSetPosition(double setpoint) {
    // Normalise setpoint
    setpoint = MathUtil.clamp(setpoint, m_armConfig.getLowerLimit(), m_armConfig.getUpperLimit());
    
    // Arm position is top if greater than halfway point, otherwise bottom
    m_armPosition = (setpoint > ARM_MIDDLE_POSITION) ? ArmPosition.Top : ArmPosition.Bottom;

    // Move arm toward setpoint
    m_armMotor.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Move arm relative to previous setpoint
   * @param setpoint Delta to move arm by (ticks)
   */
  public void armPositionRelative(double setpoint) {
    armSetPosition(m_armMotor.getClosedLoopTarget() + setpoint);
  }

  /**
   * Toggle arm between top and bottom positions
   */
  public void toggleArmPosition() {
    if (m_armPosition == ArmPosition.Top) armDown();
    else armUp();
  }

  /**
   * Move arm to top position
   */
  public void armUp(){
    armSetPosition(ArmPosition.Top.value);
  }

  /**
   * Move arm to bottom position
   */
  public void armDown(){
    armSetPosition(ArmPosition.Bottom.value);
  }

   /**
   * Intake balls
   */
  public void intake() {
    m_prevArmPosition = m_armPosition;
    armDown();
    m_rollerMotor.set(+m_rollerSpeed);
  }

  /**
   * Outtakes balls
   */
  public void outtake() {
    m_prevArmPosition = m_armPosition;
    armDown();
    m_rollerMotor.set(-m_rollerSpeed);
  }

  /**
   * Stop roller and return arm to previous position
   */
  public void stop() {
    m_rollerMotor.stopMotor();
    armSetPosition(m_prevArmPosition.value);
  }

  @Override
  public void close() {
    m_armMotor = null;
    m_rollerMotor = null;
  }
}


