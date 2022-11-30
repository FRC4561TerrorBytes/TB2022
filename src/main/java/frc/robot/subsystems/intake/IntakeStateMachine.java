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
     * @param intake the {@link IntakeSubsystem} subsystem to be operated.
     * @param rumbleController the driver controller to rumble as needed.
     */
    IntakeStateMachine(final IntakeSubsystem intake, final GenericHID rumbleController) {
        m_intake = intake;
        m_rumbleController = rumbleController;

        // Create the state commands.
        // Note that intake simulates shooter full rumble after 10 seconds.
        final Command retractedStateCommand = new InstantCommand(() -> m_intake.stop(), m_intake)
                .andThen(new WaitUntilCommand(m_intake::isExtensionRequested));
        final Command intakingStateCommand = new InstantCommand(() -> m_intake.intake(), m_intake)
                .andThen(new WaitUntilCommand(m_intake::isRetractionRequested)
                        .deadlineWith(new WaitCommand(10.0).andThen(RumbleFactory.getInstance().getGroupableCommand(m_rumbleController))));
        final Command outtakingStateCommand = new InstantCommand(() -> m_intake.outtake(), m_intake)
                .andThen(new WaitUntilCommand(m_intake::isRetractionRequested));

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
    }

    /**
     * @return the wrapper command appropriate for use as the default command.
     */
    PerpetualCommand getDefaultCommand() {
        return this.m_defaultCommand;
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
        return getNextState(State.getState(current)).ordinal();
    }

    /**
     * This is the next command state operator for the state machine. If the current
     * state is no state, the result is STOWED_EMPTY (see
     * {@link #setNextInitialState(State)} for initial state options).
     * 
     * @param currentState the current {@link State} of the state machine command.
     *                     Can be null, in which case, STOWED_EMPTY is returned.
     * 
     * @return the next {@link State} of the state machine command. Never null.
     */
    private State getNextState(final State currentState) {
        State next = State.RETRACTED;
        if (currentState != null) {
            switch (currentState) {
                case RETRACTED:
                    if (m_intake.intakeHandled()) {
                        next = State.INTAKING;
                    } else if (m_intake.outtakeHandled()) {
                        next = State.OUTTAKING;
                    }
                    break;

                case INTAKING:
                    if (m_intake.retractionHandled()) {
                        next = State.RETRACTED;
                    }
                    break;

                case OUTTAKING:
                    if (m_intake.retractionHandled()) {
                        next = State.RETRACTED;
                    }
                    break;

                default:
                    break;
            }
        }
        return next;
    }
}
