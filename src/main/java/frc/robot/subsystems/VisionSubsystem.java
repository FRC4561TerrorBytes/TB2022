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

public class VisionSubsystem extends SubsystemBase {
  public static class Hardware {
    private PhotonCamera pi;

    public Hardware(PhotonCamera pi) {
      this.pi = pi;
    }
  }

  PhotonCamera m_pi;
  PhotonPipelineResult m_latestResult;
  PhotonTrackedTarget m_latestTarget;
  double m_latestDistance;
  double m_cameraHeightMeters, m_targetHeightMeters;
  double m_cameraPitchRadians;

  /**
   * Create a new vision subsystem
   * @param visionHardware Vision hardware
   * @param cameraHeightMeters Camera height in meters
   * @param targetHeightMeters Target height in meters
   * @param cameraPitchDegrees Camera pitch in degrees
   */
  public VisionSubsystem(Hardware visionHardware, double cameraHeightMeters, double targetHeightMeters, double cameraPitchDegrees) {
    this.m_pi = visionHardware.pi;
    this.m_cameraHeightMeters = cameraHeightMeters;
    this.m_targetHeightMeters = targetHeightMeters;
    this.m_cameraPitchRadians = Units.degreesToRadians(cameraPitchDegrees);
  }

  public static Hardware initializeHardware() {
    Hardware visionHardware = new Hardware(new PhotonCamera("photonvision"));

    return visionHardware;
  }

  /**
   * Initialize VisionSubsystem
   */
  public void initialize() {
    setDriverMode(true);
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
    m_pi.setDriverMode(enable);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_pi.getDriverMode()) {
      m_latestResult = m_pi.getLatestResult();
      m_latestTarget = m_latestResult.getBestTarget();
      if (isTargetValid()) {
        m_latestDistance = PhotonUtils.calculateDistanceToTargetMeters(m_cameraHeightMeters,
                                                                       m_targetHeightMeters, 
                                                                       m_cameraPitchRadians,
                                                                       m_latestTarget.getPitch());
      } else m_latestDistance = 0.0;
    }
  }
}
