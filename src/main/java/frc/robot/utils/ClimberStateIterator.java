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

  
  public static ClimberState ClimberStart;
  public static ClimberState TelescopeUp;
  public static ClimberState TelescopeDown;
  public static ClimberState TelescopeUnhook;
  public static ClimberState WinchOut;
  public static ClimberState TelescopeReach;
  public static ClimberState WinchHook;
  public static ClimberState TelescopeGrab;
  public static ClimberState RobotSwing;
  public static ClimberState WinchIn;
  public static ClimberState FinishClimb;

  public ClimberStateIterator(double telescopeLowerLimit, double telescopeUpperLimit, double winchLowerLimit, double winchUpperLimit) {
    m_currentState = 0;

    ClimberStart = new ClimberState("Climber Start", 0, 0);
    TelescopeUp = new ClimberState("Telescope Up", -290000, 0);
    TelescopeDown = new ClimberState("Telescope Down", +295000, 0);
    TelescopeUnhook = new ClimberState("Telescope Unhook", -20000, 0);
    WinchOut = new ClimberState("Winch Out", 0, 126000);
    TelescopeReach = new ClimberState("Telescope Reach", -270000, 0);
    WinchHook = new ClimberState("Winch Hook", 0, -50000);
    TelescopeGrab = new ClimberState("Telescope Grab", +10000, 0);
    RobotSwing = new ClimberState("Robot Swing", +162000, 190000);
    WinchIn = new ClimberState("Winch In", 0, -76000);
    FinishClimb = new ClimberState("Finish Climb", +120000,0);

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
