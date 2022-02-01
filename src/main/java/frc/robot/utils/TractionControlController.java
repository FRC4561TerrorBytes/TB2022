package frc.robot.utils;

import java.util.HashMap;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class TractionControlController {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.05;

  private final double MIN_SLIP_LIMIT = 0.001;
  private final double MAX_SLIP_LIMIT = 1.0;

  private boolean m_isEnabled = true;
  private double m_deadband = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private double m_slipLimit = 0.0;
  private double m_minSlipLimit = 0.0;
  private double m_originalMinSlipLimit = 0.0;

  private HashMap<Double, Double> m_tractionControlMap = new HashMap<Double, Double>();
  private HashMap<Double, Double> m_throttleInputMap = new HashMap<Double, Double>();

  /**
   * Create an instance of TractionControlController
   * @param deadband Deadband for controller input [+0.001, +0.1]
   * @param maxLinearSpeed maximum linear speed of robot (m/s)
   * @param slipLimit acceleration limit (m/s^2)
   * @param tractionControlCurve Expression characterising traction of the robot with "X" as the variable
   * @param throttleInputCurve Expression characterising throttle input with "X" as the variable
   */
  public TractionControlController(double deadband, double maxLinearSpeed, double slipLimit, String tractionControlCurve, String throttleInputCurve) {
    m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);
    m_maxLinearSpeed = Math.floor(maxLinearSpeed * 1000) / 1000;
    m_slipLimit = MathUtil.clamp(slipLimit, MIN_SLIP_LIMIT, MAX_SLIP_LIMIT);
    m_minSlipLimit = (maxLinearSpeed / 4) * slipLimit;
    m_originalMinSlipLimit = m_minSlipLimit;

    // Get Mozilla Rhino JavaScript engine
    ScriptEngine jsEngine = new ScriptEngineManager().getEngineByName("rhino");

    // Fill traction control hashmap
    int maxSpeedCount = (int)(maxLinearSpeed * 1000.0);
    for (int i = 0; i <= maxSpeedCount; i++) {
      double key = (double)i / 1000;
      try {
        // Evaluate JavaScript, replacing "X" with value and clamp value between [0.0, +1.0]
        double value = Double.valueOf(jsEngine.eval(tractionControlCurve.replace("X", String.valueOf(key))).toString());
        value = MathUtil.clamp(value, 0.0, +1.0);
        // Add both positive and negative values to map
        m_tractionControlMap.put(+key, +value);
        m_tractionControlMap.put(-key, -value);
      } catch (ScriptException e) {
        DriverStation.reportError(e.getMessage(), true);
      }
    }

    // Fill throttle input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000;
      try {
        double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
        // Evaluate JavaScript, replacing "X" with value and clamp value between [0.0, +MAX_LINEAR_SPEED]
        double value = Double.valueOf(jsEngine.eval(throttleInputCurve.replace("X", String.valueOf(deadbandKey))).toString());
        value = MathUtil.clamp(value, 0.0, +maxLinearSpeed);
        // Add both positive and negative values to map
        m_throttleInputMap.put(+key, +value);
        m_throttleInputMap.put(-key, -value);
      } catch (ScriptException e) {
        DriverStation.reportError(e.getMessage(), true);
      }
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
    requestedAcceleration = Math.copySign(Math.floor(Math.abs(requestedAcceleration) * 100) / 100, requestedAcceleration);

    // Apply acceleration limit to requested acceleration to limit wheel slip
    double slipLimitAcceleration = Math.max(Math.abs(inertialVelocity * m_slipLimit), m_minSlipLimit);
    requestedAcceleration = Math.copySign(Math.min(Math.abs(requestedAcceleration), slipLimitAcceleration), requestedAcceleration);

    // Calculate optimal velocity and truncate value to 3 decimal places and clamp to maximum linear speed
    double velocityLookup = inertialVelocity + requestedAcceleration;
    velocityLookup = Math.copySign(Math.floor(Math.abs(velocityLookup) * 1000) / 1000, velocityLookup) + 0.0;
    velocityLookup = MathUtil.clamp(velocityLookup, -m_maxLinearSpeed, +m_maxLinearSpeed);

    return m_tractionControlMap.get(velocityLookup);
  }

  /**
   * Toggle traction control
   */
  public void toggle() {
    if (m_isEnabled) disable();
    else enable();
  }

  /**
   * Disable traction control
   */
  public void disable() {
    m_minSlipLimit = Double.MAX_VALUE;
    m_isEnabled = false;
  }

  /**
   * Enable traction control
   */
  public void enable() {
    m_minSlipLimit = m_originalMinSlipLimit;
    m_isEnabled = true;
  }

  /**
   * Check if traction control is enabled
   * @return true if enabled
   */
  public boolean isEnabled() {
    return m_isEnabled;
  }
}
