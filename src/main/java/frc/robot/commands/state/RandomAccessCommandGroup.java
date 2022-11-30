// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.state;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.IntUnaryOperator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * A CommandGroup that runs a list of commands in an arbitrary order. During
 * exection, a command included in the group may be executed 0, 1 or more times
 * and in any order. This can be useful for the implementation of state
 * machines. One common use cases are to implement a non-terminating state
 * machine that serves as the default command for a subsystem. Another is to
 * implement a terminating state machine to execute an autonomous mode that uses
 * one or more subsystems.
 *
 * <p>
 * As a rule, CommandGroups require the union of the requirements of their
 * component commands.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep TODO
 */
public class RandomAccessCommandGroup extends CommandGroupBase {
  /** The result of {@link #getCurrentCommandIndex()} if no current command. */
  public static final int NO_CURRENT_COMMAND = -1;

  /** The order commands that can be access by index (0 based). */
  private final List<Command> m_commands = new ArrayList<>();
  /** A read only wrapper for {@link #getCommands()}. */
  private final List<Command> m_unmodifiableCommands = Collections.unmodifiableList(m_commands);
  /** The operator for determining the next command. */
  private final IntUnaryOperator m_nextCommandIndexOperator;
  /** The initial command index the next time {@link #initialize()} is called. */
  private int m_initialCommandIndex = NO_CURRENT_COMMAND;
  /** The currently active command index. */
  private int m_currentCommandIndex = NO_CURRENT_COMMAND;
  /** Will continue to be true if all added commands can run when disabled. */
  private boolean m_runWhenDisabled = true;

  /**
   * Same as {@link #RandomAccessCommandGroup(IntUnaryOperator, Command...)} with
   * a null operator.
   *
   * @param commands the commands to include in this group.
   */
  public RandomAccessCommandGroup(final Command... commands) {
    this(null, commands);
  }

  /**
   * Creates a new RandomAccessCommandGroup. The given commands will be run
   * in a arbitrary order, with the CommandGroup finishing when no command is
   * selected to run next.
   * 
   * <p>
   * The only time a contained command's end() method receives true for the
   * interrupted parameter is when it is the current command and the entire group
   * is interrupted. All other contained command end() calls will be due to the
   * contained command's isFinish() returning true, and then the end() interrupted
   * parameter will be false. One important effect of this is that the interrupted
   * parameter cannot be used as reliable input into selecting the next command.
   * The isFinished() method must properly detect the time for a command change.
   * The selection of the next command is left to the
   * <code>nextCommandIndexOperator</code>.
   * 
   * <p>
   * Typically, if the nextCommandIndexOperator returned index is out of range
   * (see {@link #isCurrentCommandIndexInRange()}), this command group ends.
   * However, this command is designed to work properly with
   * {@link PerpetualCommand} (see {@link #perpetually()}). In that case, it will
   * not end, but will call the nextCommandIndexOperator during each execute call
   * with a parameter of -1.
   * 
   * <p>
   * The most common case for running this group perpetually is as a default
   * command for a subsystem and this group implements a state machine for the
   * subsystem.
   *
   * @param nextCommandIndexOperator an operator that takes the current index as a
   *                                 parameter and returns the next index. If
   *                                 null, an operator that simply increments the
   *                                 index is used. This results in behavior like
   *                                 a {@link SequentialCommandGroup}.
   * @param commands                 the commands to include in this group.
   */
  public RandomAccessCommandGroup(final IntUnaryOperator nextCommandIndexOperator, final Command... commands) {
    addCommands(commands);
    m_nextCommandIndexOperator = nextCommandIndexOperator != null ? nextCommandIndexOperator : i -> i + 1;
  }

  /**
   * {@inheritDoc}
   * 
   * @throws IllegalStateException if an attempt is made to add a command while
   *                               this command group is running.
   */
  @Override
  public void addCommands(Command... commands) {
    requireUngrouped(commands);

    if (m_currentCommandIndex != NO_CURRENT_COMMAND) {
      throw new IllegalStateException(
          "Commands cannot be added to a CommandGroup while the group is running");
    }

    // TODO unfortunately, this method is package scoped.
    // Put this back in if / when part of wpilib.
    // registerGroupedCommands(commands);

    for (Command command : commands) {
      m_commands.add(command);
      m_requirements.addAll(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
    }
  }

  /**
   * {@inheritDoc}
   * 
   * <p>
   * This implementation selects the initial command and, if found, initializes
   * it.
   */
  @Override
  public void initialize() {
    m_currentCommandIndex = m_initialCommandIndex;
    if (!isCurrentCommandIndexInRange()) {
      m_currentCommandIndex = getNextCommandIndex();
    }

    if (isCurrentCommandIndexInRange()) {
      m_commands.get(m_currentCommandIndex).initialize();
    }
  }

  /**
   * {@inheritDoc}
   * 
   * <p>
   * If the current command index is not in range, an attempt is made to get the
   * command. If it is found, it is initialized and we return. Its execution will
   * happen in the next cycle.
   * 
   * <p>
   * On the other hand, if there is a valid command active on entry, its execute
   * and isFinished calls are made. If isFinished returns true, the command's end
   * is called and the next command to execute is selected. If a valid command is
   * selected, it is initialized.
   */
  @Override
  public void execute() {
    if (!isCurrentCommandIndexInRange()) {
      // Either initialize resulted in no command or the call later in this method
      // resulted in no command and this group is being run perpetually.
      m_currentCommandIndex = getNextCommandIndex();
      if (isCurrentCommandIndexInRange()) {
        m_commands.get(m_currentCommandIndex).initialize();
      }
      return;
    }

    Command currentCommand = m_commands.get(m_currentCommandIndex);

    currentCommand.execute();
    if (currentCommand.isFinished()) {
      currentCommand.end(false);
      m_currentCommandIndex = getNextCommandIndex();
      if (isCurrentCommandIndexInRange()) {
        m_commands.get(m_currentCommandIndex).initialize();
      }
    }
  }

  /**
   * {@inheritDoc}
   * 
   * <p>
   * A {@link RandomAccessCommandGroup} is finished (return true here), when we
   * get out of execute without a valid selected command.
   */
  @Override
  public boolean isFinished() {
    return !isCurrentCommandIndexInRange();
  }

  /**
   * {@inheritDoc}
   * 
   * <p>
   * If interrupted is true and we have a valid command, call the command's end
   * with true (only way this will happen). If not interrupted, there is no valid
   * command to end as detected in isFinished.
   */
  @Override
  public void end(final boolean interrupted) {
    if (interrupted && isCurrentCommandIndexInRange()) {
      m_commands.get(m_currentCommandIndex).end(true);
    }
    m_currentCommandIndex = NO_CURRENT_COMMAND;
  }

  /**
   * {@inheritDoc}
   * 
   * <p>
   * This command group runs when disabled if all of its contained commands can
   * run when disabled. This boolean is calculated in
   * {@link #addCommands(Command...)}.
   */
  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  /**
   * Sets the index of the command to use the next time the command group is
   * initialized. Setting this attribute to {@link #NO_CURRENT_COMMAND} directs
   * the {@link #initialize()} to call the {@link #m_nextCommandIndexOperator} to
   * get the initial command index (NO_CURRENT_COMMAND is the default value).
   * 
   * @param initialCommandIndex the index of the command to use the next time the
   *                            command group is initialized. Used as is if in
   *                            range. Set to {@link #NO_CURRENT_COMMAND} if
   *                            parameter is out of range.
   */
  public void setInitialCommandIndex(final int initialCommandIndex) {
    m_initialCommandIndex = initialCommandIndex >= 0 && initialCommandIndex < m_commands.size()
        ? initialCommandIndex
        : NO_CURRENT_COMMAND;
  }

  /**
   * @return the current command index. It will be set to
   *         {@link #NO_CURRENT_COMMAND} if out of range.
   */
  public final boolean isCurrentCommandIndexInRange() {
    boolean inRange = m_currentCommandIndex >= 0 && m_currentCommandIndex < m_commands.size();
    if (!inRange) {
      m_currentCommandIndex = NO_CURRENT_COMMAND;
    }
    return inRange;
  }

  /**
   * @return the next command index. It will have already been set in
   *         {@link #m_currentCommandIndex} and set to {@link #NO_CURRENT_COMMAND}
   *         if out of range.
   */
  protected int getNextCommandIndex() {
    m_currentCommandIndex = m_nextCommandIndexOperator.applyAsInt(m_currentCommandIndex);
    isCurrentCommandIndexInRange();
    return m_currentCommandIndex;
  }

  /**
   * @return the current command index. If no command is currently running,
   *         {@link #NO_CURRENT_COMMAND} is returned.
   */
  public final int getCurrentCommandIndex() {
    isCurrentCommandIndexInRange();
    return m_currentCommandIndex;
  }

  /**
   * @return an unmodifiable wrapper around the list of commands in this group.
   */
  public final List<Command> getCommands() {
    return m_unmodifiableCommands;
  }
}
