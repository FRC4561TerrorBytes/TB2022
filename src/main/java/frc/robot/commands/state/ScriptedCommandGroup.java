// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;

/**
 * An extension of {@link RandomAccessCommandGroup} that runs a script of
 * command indexes. The script can be changed whenever the command group is not
 * running. The currently set script will be used on the next command group
 * execution. The default script is empty.
 * 
 * <p>
 * Note that the script may contain a given command index 0, 1, or more times.
 * 
 * <p>
 * The primary motivation for this class is to execute a terminating command
 * group. This is particularly useful for autonomous commands or short lived
 * bits of automation used during teleop. Since the script can be changed
 * between runs, this class is particularly useful for automation that uses a
 * known set of steps but the order of steps may need to vary based on
 * environmental conditions. For example, the used of known steps during
 * autonomous mode that varies based on sensed field indicators at the start of
 * the match.
 * 
 * <p>
 * As a rule, CommandGroups require the union of the requirements of their
 * component commands.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep TODO
 */
public class ScriptedCommandGroup extends RandomAccessCommandGroup {
    public static final int[] EMPTY_SCRIPT = {};
    private int[] m_script = EMPTY_SCRIPT;
    private int m_currentScriptIndex = RandomAccessCommandGroup.NO_CURRENT_COMMAND;

    /**
     * Creates a new ScriptedCommandGroup. The given commands will be run in a
     * scripted order, with the CommandGroup finishing when no command is
     * selected to run next, that is, the end of the script is reached.
     * 
     * <p>
     * Note that execution always starts at the beginning of the script. Therefore,
     * the {@link #setInitialCommandIndex(int)} is overridden to do nothing.
     * 
     * <p>
     * The only time a contained command's end() method receives true for the
     * interrupted parameter is when it is the current command and the entire group
     * is interrupted. All other contained command end() calls will be due to the
     * contained command's isFinish() returning true, and then the end() interrupted
     * parameter will be false.
     * 
     * <p>
     * Typically, if the end of the script is reached, this command group ends.
     * However, this command is designed to work properly with
     * {@link PerpetualCommand} (see {@link #perpetually()}). In that case, it will
     * be restarted from the beginning of the script.
     *
     * @param commands the commands to include in this group.
     */
    public ScriptedCommandGroup(final Command... commands) {
        // Would prefer to send an IntUnaryOperator on super call, but that would
        // require an instance method reference during construction which is not
        // allowed. See the getNextCommandIndex() override instead.
        super(commands);
    }

    /**
     * Uses the provided script and current script index to update the script index
     * and retrieve the next command index from the script.
     */
    @Override
    protected int getNextCommandIndex() {
        m_currentScriptIndex++;
        if (m_currentScriptIndex >= m_script.length) {
            m_currentScriptIndex = RandomAccessCommandGroup.NO_CURRENT_COMMAND;
            return RandomAccessCommandGroup.NO_CURRENT_COMMAND;
        }
        return m_script[m_currentScriptIndex];
    }

    /**
     * Sets the script to use the next time the script is run.
     * 
     * @param script the next script (an array of valid command indicies).
     * 
     * @throws IllegalStateException if the command group is running.
     */
    public void setScript(final int[] script) {
        if (isScheduled()) {
            throw new IllegalStateException("Cannot change the script while running.");
        }
        m_script = script;
        if (m_script == null) {
            m_script = EMPTY_SCRIPT;
        }
    }

    /**
     * This method does nothing since script indexing always starts at 0 and will
     * use the command index found there. If a different start point is desired, set
     * a different script before scheduling.
     */
    @Override
    public void setInitialCommandIndex(final int initialCommandIndex) {
    }
}
