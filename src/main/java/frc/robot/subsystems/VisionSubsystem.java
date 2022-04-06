// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  public static class Hardware {
    private PhotonCamera pi, webcam;

    public Hardware(PhotonCamera pi, PhotonCamera webcam) {
      this.pi = pi;
      this.webcam = webcam;
    }
  }

  private final double MAX_TOLERANCE = 7.5;
  private final double MIN_TOLERANCE = 2.0;
  private final int VISION_PIPELINE_INDEX = 0;

  private PhotonCamera m_pi;
  private PhotonCamera m_webcam;
  private PhotonPipelineResult m_latestResult;
  private PhotonTrackedTarget m_latestTarget;
  private double m_latestDistance;
  private double m_cameraHeightMeters, m_targetHeightMeters;
  private double m_cameraPitchRadians;
  private double m_toleranceSlope;

  /**
   * Create a new vision subsystem
   * @param visionHardware Vision hardware
   * @param cameraHeightMeters Camera height in meters
   * @param targetHeightMeters Target height in meters
   * @param cameraPitchDegrees Camera pitch in degrees
   * @param maxDistance Max vision shooting distance in meters
   */
  public VisionSubsystem(Hardware visionHardware, double cameraHeightMeters, double targetHeightMeters, double cameraPitchDegrees, double maxDistance) {
    this.m_pi = visionHardware.pi;
    this.m_webcam = visionHardware.webcam;
    this.m_cameraHeightMeters = Constants.CAMERA_HEIGHT_METERS;
    this.m_targetHeightMeters = Constants.TARGET_HEIGHT_METERS;
    this.m_cameraPitchRadians = Units.degreesToRadians(Constants.CAMERA_PITCH_DEGREES);
    this.m_toleranceSlope = -MAX_TOLERANCE / maxDistance;
  }

  /**
   * Get aiming tolerance for current distance
   * @return aiming tolerance in degrees
   */
  private double getTolerance() {
    return Math.max(getDistance() * m_toleranceSlope + MAX_TOLERANCE, MIN_TOLERANCE);
  }

  public static Hardware initializeHardware() {
    Hardware visionHardware = new Hardware(new PhotonCamera("photonvision"), new PhotonCamera("webcam"));

    return visionHardware;
  }

  /**
   * Initialize VisionSubsystem
   */
  public void initialize() {
    setDriverMode(false);
  }

  /**
   * Initialize VisionSubsystem before disable
   */
  public void disabledInit() {
    setDriverMode(true);
  }

  /**
   * Enable or disable driver mode
   * @param enable Whether to set driver mode
   */
  public void setDriverMode(boolean enable) {
    // webcam is always in driver mode
    m_webcam.setDriverMode(true);

    // only toggle pi
    m_pi.setDriverMode(enable);
    if (enable) m_pi.setPipelineIndex(VISION_PIPELINE_INDEX);
  }

  /**
   * Get driver mode
   * @return true if driver mode enabled
   */
  public boolean getDriverMode() {
    return m_pi.getDriverMode();
  }

  /**
   * Return latest target
   * @return Latest best target
   */
  public PhotonTrackedTarget getTarget() {
    return m_latestTarget;
  }

  /**
   * If PhotonVision has found a valid target
   * @return true if target has been identified
   */
  public boolean isTargetValid() {
    return m_latestTarget != null;
  }

  /**
   * Get yaw angle to target
   * @return yaw angle to best target in degrees
   */
  public double getYaw() {
    if (isTargetValid()) return m_latestTarget.getYaw();
    else return 0.0;
  }

  /**
   * Get latest range
   * @return range to best target in meters
   */
  public double getDistance() {
    return m_latestDistance;
  }

  /**
   * If PhotonVision is lined up on target
   * @return true if aimed at target within tolerance
   */
  public boolean isOnTarget() {
    return isTargetValid() && Math.abs(m_latestTarget.getYaw()) < getTolerance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!getDriverMode()) {
      m_latestResult = m_pi.getLatestResult();
      m_latestTarget = m_latestResult.getBestTarget();
      if (isTargetValid()) {
        m_latestDistance = PhotonUtils.calculateDistanceToTargetMeters(m_cameraHeightMeters,
                                                                       m_targetHeightMeters, 
                                                                       m_cameraPitchRadians,
                                                                       Units.degreesToRadians(m_latestTarget.getPitch()));
        System.out.println("Vision Yaw: " + getYaw());
        System.out.println("Vision Tolerance: " + getTolerance());
      } else m_latestDistance = 0.0;
    } else m_latestTarget = null;
    System.out.println("Vision Distance: " + getDistance());
  }
}
