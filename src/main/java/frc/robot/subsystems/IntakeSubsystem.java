// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;


public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private WPI_TalonFX armMotor;
    private WPI_TalonFX rollerMotor;

    public Hardware(WPI_TalonFX armMotor, 
                    WPI_TalonFX rollerMotor) {
      this.armMotor = armMotor;
      this.rollerMotor = rollerMotor;
    }

  }


  private String SUBSYSTEM_NAME = "Intake Subsystem";

  private WPI_TalonFX m_armMotor;
  private WPI_TalonFX m_rollerMotor;
  private TalonPIDConfig m_armConfig;


   /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Hardware intakeHardWare, TalonPIDConfig armConfig) {
    this.m_armMotor = intakeHardWare.armMotor;
    this.m_rollerMotor = intakeHardWare.rollerMotor;
    this.m_armConfig = armConfig;
  }

  public static Hardware initializeHardware() {
    Hardware intakeHardWare = new Hardware(new WPI_TalonFX(Constants.ARM_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.INTAKE_ROLLER_PORT));

    return intakeHardWare;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armPositionRelative(double setpoint) {
    armSetPosition(m_armMotor.getClosedLoopTarget() + setpoint);
  }

  public void armManual(double speed) {
    m_armMotor.set(speed);
  }

  public void armSetPosition(double setpoint) {
    // normalise setpoint
    if (setpoint < m_armConfig.getLowerLimit()) setpoint = m_armConfig.getLowerLimit();
    if (setpoint > m_armConfig.getUpperLimit()) setpoint = m_armConfig.getUpperLimit();

    // Move arm toward setpoint
    m_armMotor.set(ControlMode.MotionMagic, setpoint);
  }

  public void toggleArmPosition() {
    if (m_armMotor.getClosedLoopTarget() == Constants.ARM_TOP_POSITION) {
      armSetPosition(Constants.ARM_BOTTOM_POSITION);
    } else if (m_armMotor.getClosedLoopTarget() == Constants.ARM_BOTTOM_POSITION) {
      armSetPosition(Constants.ARM_TOP_POSITION);
    }
  }

  @Override
  public void close() throws Exception {
    m_armMotor = null;
    m_rollerMotor = null;
  }
}


