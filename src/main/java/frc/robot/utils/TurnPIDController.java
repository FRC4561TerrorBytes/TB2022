package frc.robot.utils;

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class TurnPIDController extends PIDController {
  private HashMap<Double, Double> m_turnInputMap = new HashMap<Double, Double>();
  private double m_turnScalar = 0.0;
  private double m_deadband = 0.0;
  private boolean m_isTurning = false;

  public TurnPIDController(double kp, double kd, double turnScalar, double deadband, PolynomialSplineFunction turnInputCurve) {
    super(kp, 0.0, kd, Constants.ROBOT_LOOP_PERIOD);
    this.m_turnScalar = turnScalar;
    this.m_deadband = deadband;

    // Fill turn input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000; 
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate and clamp value between [0.0, +1.0]
      double value = MathUtil.clamp(turnInputCurve.value(deadbandKey), 0.0, +1.0);
      // Add both positive and negative values to map
      m_turnInputMap.put(+key, +value);
      m_turnInputMap.put(-key, -value);
    }
  }

  /**
   * Returns next output of TurnPIDController
   * @param currentAngle current yaw angle of robot (degrees)
   * @param turnRequest turn request [-1.0, +1.0]
   * 
  * @return optimal turn output [-1.0, +1.0]
   */
  public double calculate(double currentAngle, double turnRequest) {
    // Start turning if input is greater than deadband
    if (Math.abs(turnRequest) >= m_deadband) {
      // Get scaled turnRequest
      double scaledTurnRequest = m_turnInputMap.get(turnRequest);
      // Add delta to setpoint scaled by factor
      super.setSetpoint(currentAngle + (scaledTurnRequest * m_turnScalar));
      m_isTurning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (m_isTurning) {
        super.setSetpoint(currentAngle);
        m_isTurning = false;
      }
    }

    return MathUtil.clamp(super.calculate(currentAngle), -1.0, +1.0);
  }

  public boolean isTurning() {
    return m_isTurning;
  }
}
