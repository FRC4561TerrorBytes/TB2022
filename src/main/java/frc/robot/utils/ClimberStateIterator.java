package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class ClimberStateIterator {

  public static class ClimberState {
    private final String name;
    private final double telescopePosition;
    private final double winchPosition;

    private ClimberState(String name, double telescopePosition, double winchPosition) {
      this.name = name;
      this.telescopePosition = telescopePosition;
      this.winchPosition = winchPosition;
    }

    public String getName() {
      return name;
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

    ClimberStart = new ClimberState("Climber Start", telescopeLowerLimit, winchLowerLimit);
    TelescopeUp = new ClimberState("Telescope Up", telescopeUpperLimit, winchLowerLimit);
    TelescopeDown = new ClimberState("Telescope Down", telescopeLowerLimit, winchLowerLimit);
    TelescopeUnhook = new ClimberState("Telescope Unhook", telescopeUpperLimit * 0.1, winchLowerLimit);
    WinchOut = new ClimberState("Winch Out", telescopeUpperLimit * 0.1, winchUpperLimit * 0.5);
    TelescopeReach = new ClimberState("Telescope Reach", telescopeUpperLimit, winchUpperLimit * 0.5);
    WinchHook = new ClimberState("Winch Hook", telescopeUpperLimit, winchUpperLimit * 0.25);
    TelescopeGrab = new ClimberState("Telescope Grab", telescopeUpperLimit * 0.75, winchUpperLimit * 0.25);
    RobotSwing = new ClimberState("Robot Swing", telescopeUpperLimit * 0.5, winchUpperLimit);
    WinchIn = new ClimberState("Winch In", telescopeUpperLimit * 0.5, winchLowerLimit);
    FinishClimb = new ClimberState("Finish Climb", telescopeLowerLimit, winchLowerLimit);

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
    if (m_currentState > 10) m_currentState = 3;
  }

  public void previousState() {
    m_currentState--;
    if (m_currentState < 0) m_currentState = 0;
  }

  public ClimberState previewPreviousState() {
    int previousState = m_currentState - 1;
    if (previousState < 0) previousState = 0;
    
    return m_states.get(previousState);
  }

  public ClimberState previewNextState() {
    int nextState = m_currentState + 1;
    if (nextState > 10) nextState = 3;

    return m_states.get(nextState);
  }
}
