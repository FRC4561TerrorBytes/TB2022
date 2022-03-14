package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class ClimberStateIterator {

  public static class ClimberState {
    private final double telescopePosition;
    private final double winchPosition;

    private ClimberState(double telescopePosition, double winchPosition) {
      this.telescopePosition = telescopePosition;
      this.winchPosition = winchPosition;
    }

    public double getTelescopePosition() {
      return telescopePosition;
    }

    public double getWinchPosition() {
      return winchPosition;
    }
  }

  private ArrayList<ClimberState> m_states;
  private int m_currentState = 0;

  
  private static ClimberState ClimberStart;
  private static ClimberState TelescopeUp;
  private static ClimberState TelescopeDown;
  private static ClimberState TelescopeUnhook;
  private static ClimberState WinchOut;
  private static ClimberState TelescopeReach;
  private static ClimberState WinchHook;
  private static ClimberState TelescopeGrab;
  private static ClimberState RobotSwing;
  private static ClimberState WinchIn;
  private static ClimberState FinishClimb;

  public ClimberStateIterator(double telescopeLowerLimit, double telescopeUpperLimit, double winchLowerLimit, double winchUpperLimit) {
    m_currentState = 0;

    ClimberStart = new ClimberState(telescopeLowerLimit, winchLowerLimit);
    TelescopeUp = new ClimberState(telescopeUpperLimit, winchLowerLimit);
    TelescopeDown = new ClimberState(telescopeLowerLimit, winchLowerLimit);
    TelescopeUnhook = new ClimberState(telescopeUpperLimit * 0.1, winchLowerLimit);
    WinchOut = new ClimberState(telescopeUpperLimit * 0.1, winchUpperLimit * 0.5);
    TelescopeReach = new ClimberState(telescopeUpperLimit, winchUpperLimit * 0.5);
    WinchHook = new ClimberState(telescopeUpperLimit, winchUpperLimit * 0.25);
    TelescopeGrab = new ClimberState(telescopeUpperLimit * 0.75, winchUpperLimit * 0.25);
    RobotSwing = new ClimberState(telescopeUpperLimit * 0.5, winchUpperLimit);
    WinchIn = new ClimberState(telescopeUpperLimit * 0.5, winchLowerLimit);
    FinishClimb = new ClimberState(telescopeLowerLimit, winchLowerLimit);

    m_states = new ArrayList<ClimberState>(List.of(ClimberStart,
                                                   TelescopeUp,
                                                   TelescopeDown,
                                                   TelescopeUnhook,
                                                   WinchOut,
                                                   TelescopeReach,
                                                   WinchHook,
                                                   TelescopeGrab,
                                                   RobotSwing,
                                                   WinchIn,
                                                   FinishClimb));
  }

  public ClimberState getCurrentState() {
    return m_states.get(m_currentState);
  }

  public void nextState() {
    m_currentState++;
    if (m_currentState > 10) {
      m_currentState = 3;
    }
  }

  public void previousState() {
    m_currentState--;
    if (m_currentState < 0) {
      m_currentState = 0;
    }
  }

}
