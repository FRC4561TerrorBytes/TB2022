package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class ClimberStateIterator {

  public static class ClimberState {
    private final int telescopePosition;
    private final int winchPosition;

    private ClimberState(int telescopePosition, int winchPosition) {
      this.telescopePosition = telescopePosition;
      this.winchPosition = winchPosition;
    }

    public int getTelescopePosition() {
      return telescopePosition;
    }

    public int getWinchPosition() {
      return winchPosition;
    }
  }

  private ArrayList<ClimberState> states;
  private int currentState = 0;

  public static final ClimberState ClimberStart = new ClimberState(0, 0);
  public static final ClimberState TelescopeUp = new ClimberState(0, 0);
  public static final ClimberState TelescopeDown = new ClimberState(0, 0);
  public static final ClimberState TelescopeUnhook = new ClimberState(0, 0);
  public static final ClimberState WinchOut = new ClimberState(0, 0);
  public static final ClimberState TelescopeReach = new ClimberState(0, 0);
  public static final ClimberState WinchHook = new ClimberState(0, 0);
  public static final ClimberState TelescopeGrab = new ClimberState(0, 0);
  public static final ClimberState RobotSwing = new ClimberState(0, 0);
  public static final ClimberState WinchIn = new ClimberState(0, 0);
  public static final ClimberState FinishClimb = new ClimberState(0, 0);

  public ClimberStateIterator() {
    states = new ArrayList<ClimberState>(List.of(ClimberStart,
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
    currentState = 0;
  }

  public ClimberState getCurrentState() {
    return states.get(currentState);
  }

  public void nextState() {
    currentState++;
    if (currentState > 10) {
      currentState = 3;
    }
  }

  public void previousState() {
    currentState--;
    if (currentState < 0) {
      currentState = 0;
    }
  }

}
