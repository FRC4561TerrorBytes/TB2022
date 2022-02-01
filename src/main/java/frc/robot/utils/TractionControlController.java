package frc.robot.utils;

import java.util.HashMap;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class TractionControlController {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.1;

  private final double MIN_SLIP_RATIO = 0.001;
  private final double MAX_SLIP_RATIO = 0.999;

  private final double VELOCITY_THRESHOLD = 0.1;

  private boolean m_isEnabled = true;
  private double m_deadband = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private double m_slipRatio = 0.0;
  private double m_minSlipRatio = 0.0;

  private HashMap<Double, Double> m_tractionControlMap = new HashMap<Double, Double>();
  private HashMap<Double, Double> m_throttleInputMap = new HashMap<Double, Double>();

  /**
   * Create an instance of TractionControlController
   * @param deadband Deadband for controller input [+0.001, +0.1]
   * @param maxLinearSpeed maximum linear speed of robot (m/s)
   * @param slipRatio Optimal slip ratio (%) [+0.001, +1.0]
   * @param tractionControlCurve Expression characterising traction of the robot with "X" as the variable
   * @param throttleInputCurve Expression characterising throttle input with "X" as the variable
   */
  public TractionControlController(double deadband, double maxLinearSpeed, double slipRatio, String tractionControlCurve, String throttleInputCurve) {
    m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);
    m_maxLinearSpeed = Math.floor(maxLinearSpeed * 1000) / 1000;
    m_slipRatio = MathUtil.clamp(slipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    m_minSlipRatio = (VELOCITY_THRESHOLD / (1 - m_slipRatio)) / m_maxLinearSpeed;

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

    // Apply slip ratio to requested acceleration to limit wheel slip
    if (m_isEnabled) {
      requestedAcceleration = Math.abs(inertialVelocity) > VELOCITY_THRESHOLD ? 
                              Math.copySign((inertialVelocity / (1 - m_slipRatio)) - inertialVelocity, requestedAcceleration) :
                              requestedAcceleration * m_minSlipRatio;
    }

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
    m_isEnabled = false;
  }

  /**
   * Enable traction control
   */
  public void enable() {
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
