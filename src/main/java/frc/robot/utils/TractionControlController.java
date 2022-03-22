package frc.robot.utils;

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.MathUtil;

public class TractionControlController {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.15;

  private double m_deadband = 0.0;
  private double m_maxLinearSpeed = 0.0;

  private HashMap<Double, Double> m_tractionControlMap = new HashMap<Double, Double>();
  private HashMap<Double, Double> m_throttleInputMap = new HashMap<Double, Double>();

  /**
   * Create an instance of TractionControlController
   * @param deadband Deadband for controller input [+0.001, +0.1]
   * @param maxLinearSpeed maximum linear speed of robot (m/s)
   * @param tractionControlCurve Expression characterising traction of the robot with "X" as the variable
   * @param throttleInputCurve Expression characterising throttle input with "X" as the variable
   */
  public TractionControlController(double deadband, double maxLinearSpeed, PolynomialSplineFunction tractionControlCurve, PolynomialSplineFunction throttleInputCurve) {
    m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);
    m_maxLinearSpeed = Math.floor(maxLinearSpeed * 1000) / 1000;

    // Fill traction control hashmap
    int maxSpeedCount = (int)(maxLinearSpeed * 1000.0);
    for (int i = 0; i <= maxSpeedCount; i++) {
      double key = (double)i / 1000;
      // Evaluate and clamp value between [0.0, +1.0]
      double value = MathUtil.clamp(tractionControlCurve.value(key), 0.0, +1.0);
      // Add both positive and negative values to map
      m_tractionControlMap.put(+key, +value);
      m_tractionControlMap.put(-key, -value);
    }

    // Fill throttle input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000;
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate value between [0.0, +MAX_LINEAR_SPEED]
      double value = MathUtil.clamp(throttleInputCurve.value(deadbandKey), 0.0, +maxLinearSpeed);
      // Add both positive and negative values to map
      m_throttleInputMap.put(+key, +value);
      m_throttleInputMap.put(-key, -value);
    }
  }

  /**
   * Returns the next output of the traction control controller
   * @param inertialVelocity Current inertial velocity
   * @param speedRequest Speed request [-1.0, +1.0]
   * @return Optimal motor speed output [-1.0, +1.0]
   */
  public double calculate(double inertialVelocity, double speedRequest) {
    // Set drive speed if it is more than the deadband
    speedRequest = Math.copySign(Math.floor(Math.abs(speedRequest) * 1000) / 1000, speedRequest) + 0.0;
    double requestedLinearSpeed = m_throttleInputMap.get(speedRequest);

    // Get requested acceleration with minimum of 1 cm/sec^2
    double requestedAcceleration = requestedLinearSpeed - inertialVelocity;

    // Calculate optimal velocity and truncate value to 3 decimal places and clamp to maximum linear speed
    double velocityLookup = inertialVelocity + requestedAcceleration;
    velocityLookup = Math.copySign(Math.floor(Math.abs(velocityLookup) * 1000) / 1000, velocityLookup) + 0.0;
    velocityLookup = MathUtil.clamp(velocityLookup, -m_maxLinearSpeed, +m_maxLinearSpeed);

    return m_tractionControlMap.get(velocityLookup);
  }
}
