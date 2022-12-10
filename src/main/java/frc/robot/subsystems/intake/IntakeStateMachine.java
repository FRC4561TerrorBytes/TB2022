// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.state.RandomAccessCommandGroup;
import frc.robot.utils.RumbleFactory;

/**
 * This class defines the state machine for automated operation of the intake
 * including a {@link PerpetualCommand} wrapper around the
 * {@link RandomAccessCommandGroup} used to implement the state machine. The
 * wrapper can be used as the default command for the intake subsystem.
 * 
 * <p>
 * Note that this class and its content is at most package scoped (never
 * public). It must only be used directly by the intake subsystem.
 */
public class IntakeStateMachine {
    /** The intake this state machine operates. */
    private final IntakeSubsystem m_intake;
    /** The driver controller to rumble as needed. */
    private final GenericHID m_rumbleController;
    /** The validated next State. */
    private State m_nextState = null;

    /**
     * This enumeration defines the state names for the machine. The order is
     * important for proper state transition.
     * 
     * <p>
     * Note: I would have liked to make each state's command a member of the enum
     * itself, but that would involve non-static references from a static context,
     * which is not possible.
     */
    enum State {
        RETRACTED,
        INTAKING,
        OUTTAKING;

        /**
         * For enum/ordinal mapping and bounds checking, the values array is useful.
         * However, each call to the values() method creates a new array and we use it
         * quite often. Stash a single copy here and use it at all times as it will
         * never change.
         */
        private static final State VALUES[] = values();

        /**
         * Return the enumerator with the given ordinal. If the ordinal is out of range,
         * null is returned.
         * 
         * @param ordinal the ordinal of the enumerator being requested.
         * @return the enumerator or null if the ordinal is out of range.
         */
        private static State getState(final int ordinal) {
            if (ordinal >= 0 && ordinal < VALUES.length) {
                return VALUES[ordinal];
            }
            return null;
        }
    }

    /** The state machine implementation command group. */
    private final RandomAccessCommandGroup m_stateMachineCommand;

    /**
     * A wrapper for default command setting that runs the machine perpetually. This
     * makes the machine satisfy the subsystem default command requirement that
     * isFinished always returns false.
     */
    private final PerpetualCommand m_defaultCommand;

    /**
     * @param intake           the {@link IntakeSubsystem} subsystem to be operated.
     * @param rumbleController the driver controller to rumble as needed.
     */
    IntakeStateMachine(final IntakeSubsystem intake, final GenericHID rumbleController) {
        m_intake = intake;
        m_rumbleController = rumbleController;

        // Create the state commands.
        // Note that intake simulates shooter full rumble after 10 seconds.
        final Command retractedStateCommand = new InstantCommand(() -> m_intake.stop(), m_intake)
                .andThen(new WaitUntilCommand(this::isNextStatePending));
        final Command intakingStateCommand = new InstantCommand(() -> m_intake.intake(), m_intake)
                .andThen(new WaitUntilCommand(this::isNextStatePending)
                        .deadlineWith(new WaitCommand(10.0)
                                .andThen(RumbleFactory.getInstance().getGroupableCommand(m_rumbleController))));
        final Command outtakingStateCommand = new InstantCommand(() -> m_intake.outtake(), m_intake)
                .andThen(new WaitUntilCommand(this::isNextStatePending));

        /*
         * Create the state machine implementing command group. Make sure the commands
         * are in the same order as the states in the State enumeration.
         */
        m_stateMachineCommand = new RandomAccessCommandGroup(
                this::getNextStateIndex,
                retractedStateCommand,
                intakingStateCommand,
                outtakingStateCommand);

        m_defaultCommand = m_stateMachineCommand.perpetually();

        new Trigger(this::isStateMachineRunning)
                .whenInactive(new InstantCommand(() -> m_nextState = null));
    }

    /**
     * @return the wrapper command appropriate for use as the default command.
     */
    PerpetualCommand getDefaultCommand() {
        return this.m_defaultCommand;
    }

    /**
     * @return the current {@link State} or null if no state active.
     */
    State getCurrentState() {
        final int currentIndex = m_stateMachineCommand.getCurrentCommandIndex();
        return currentIndex == RandomAccessCommandGroup.NO_CURRENT_COMMAND
                ? null
                : State.getState(currentIndex);
    }

    /**
     * This is the next command index operator for the state machine. If the current
     * state is no state, the result is STOWED_EMPTY (see
     * {@link #setNextInitialState(State)} for initial state options).
     * 
     * <p>
     * For the intake, the state machine is rather complex and is implemented in
     * {@link #getNextState(State)}. That method is wrapped by this method.
     * This method handles enum/ordinal mapping so that the implemenation can be
     * purely enum based for readability.
     * 
     * @param current the current state index as passed from the state machine
     *                command.
     * 
     * @return the next state index in the range [0, State.SIZE).
     */
    private int getNextStateIndex(final int current) {
        return getNextState().ordinal();
    }

    /**
     * This is the next command state operator for the state machine. If the pending
     * next state is no state, the result is RETRACTED.
     * 
     * @return the next {@link State} of the state machine command. Never null.
     */
    synchronized private State getNextState() {
        final State next = m_nextState == null ? State.RETRACTED : m_nextState;
        m_nextState = null;
        return next;
    }

    /**
     * @return true if the state machine is running and false otherwise.
     */
    public boolean isStateMachineRunning() {
        return this.getDefaultCommand().isScheduled();
    }

    /**
     * @return true if a next statevalidated and ready for activation.
     */
    boolean isNextStatePending() {
        return m_nextState != null;
    }

    /**
     * Called to request that we start the intake of cargo.
     * 
     * @return true if the request was granted.
     */
    synchronized boolean requestIntake() {
        boolean stateChange = false;
        final State current = getCurrentState();
        if ((current != null) && (!isNextStatePending())) {
            // State machine is running and in valid state
            // with no pending state change.
            if ((current == State.RETRACTED) && (!m_intake.isFull())) {
                m_nextState = State.INTAKING;
                stateChange = true;
            }
        }
        return stateChange;
    }

    /**
     * Called to request that we start the outtake of cargo.
     * 
     * @return true if the request was granted.
     */
    public boolean requestOuttake() {
        boolean stateChange = false;
        final State current = getCurrentState();
        if ((current != null) && (!isNextStatePending())) {
            // State machine is running and in valid state
            // with no pending state change.
            if (current == State.RETRACTED) {
                m_nextState = State.OUTTAKING;
                stateChange = true;
            }
        }
        return stateChange;
    }

    /**
     * Called to request that the intake be retracted.
     * 
     * @return true if the request was granted.
     */
    public boolean requestRetraction() {
        boolean stateChange = false;
        final State current = getCurrentState();
        if ((current != null) && (!isNextStatePending())) {
            // State machine is running and in valid state
            // with no pending state change.
            if ((current == State.INTAKING) || (current == State.OUTTAKING)) {
                m_nextState = State.RETRACTED;
                stateChange = true;
            }
        }
        return stateChange;
    }
}
