// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.concurrent.atomic.AtomicReference;

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
    /** Used to control the intake of cargo (see {@link IntakeRequest}). */
    private final AtomicReference<IntakeRequest> m_request = new AtomicReference<>(IntakeRequest.NO_REQUEST_PENDING);

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

    /**
     * This enum describes the steps in handling the extension and retraction of the
     * intake.
     */
    private enum IntakeRequest {
        /** No current state change request. */
        NO_REQUEST_PENDING,
        /** Extension and intake requested. */
        INTAKE_REQUESTED,
        /** Extension and outtake requested. */
        OUTTAKE_REQUESTED,
        /** Retraction and stop of the intake has been requested. */
        RETRACTION_REQUESTED;
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
                .andThen(new WaitUntilCommand(this::isExtensionRequested));
        final Command intakingStateCommand = new InstantCommand(() -> m_intake.intake(), m_intake)
                .andThen(new WaitUntilCommand(this::isRetractionRequested)
                        .deadlineWith(new WaitCommand(10.0)
                                .andThen(RumbleFactory.getInstance().getGroupableCommand(m_rumbleController))));
        final Command outtakingStateCommand = new InstantCommand(() -> m_intake.outtake(), m_intake)
                .andThen(new WaitUntilCommand(this::isRetractionRequested));

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
                .whenInactive(new InstantCommand(() -> m_request.set(IntakeRequest.NO_REQUEST_PENDING)));
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
                    if (intakeHandled()) {
                        next = State.INTAKING;
                    } else if (outtakeHandled()) {
                        next = State.OUTTAKING;
                    }
                    break;

                case INTAKING:
                    if (retractionHandled()) {
                        next = State.RETRACTED;
                    }
                    break;

                case OUTTAKING:
                    if (retractionHandled()) {
                        next = State.RETRACTED;
                    }
                    break;

                default:
                    break;
            }
        }
        return next;
    }

    /**
     * @return true if the state machine is running and false otherwise.
     */
    public boolean isStateMachineRunning() {
        return this.getDefaultCommand().isScheduled();
    }

    /**
     * Called to request that we start the intake of cargo.
     * 
     * @return true if the request was granted (currently retracted).
     */
    public boolean requestIntake() {
        return isStateMachineRunning()
                && m_intake.isArmRetracted()
                && m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.INTAKE_REQUESTED);
    }

    /**
     * Called to request that we start the outtake of cargo.
     * 
     * @return true if the request was granted (currently retracted).
     */
    public boolean requestOuttake() {
        return isStateMachineRunning()
                && m_intake.isArmRetracted()
                && m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.OUTTAKE_REQUESTED);
    }

    /**
     * Called to request that the intake be retracted.
     * 
     * @return true if the request was granted.
     */
    public boolean requestRetraction() {
        return isStateMachineRunning()
                && !m_intake.isArmRetracted()
                && m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.RETRACTION_REQUESTED);
    }

    /**
     * @return true if intake has been requested but not yet handled.
     */
    boolean isIntakeRequested() {
        return m_request.get() == IntakeRequest.INTAKE_REQUESTED;
    }

    /**
     * @return true if intake has been requested but not yet handled.
     */
    boolean isOuttakeRequested() {
        return m_request.get() == IntakeRequest.OUTTAKE_REQUESTED;
    }

    /**
     * @return true if either intake or outake has been requested but not yet
     *         handled.
     */
    boolean isExtensionRequested() {
        return isIntakeRequested() || isOuttakeRequested();
    }

    /**
     * @return true if intake retraction has been requested but not yet handled.
     */
    boolean isRetractionRequested() {
        return m_request.get() == IntakeRequest.RETRACTION_REQUESTED;
    }

    /**
     * @return true (the caller can move to next state machine state) if an
     *         intake had been requested.
     */
    boolean intakeHandled() {
        return m_request.compareAndSet(IntakeRequest.INTAKE_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
    }

    /**
     * @return true (the caller can move to next state machine state) if an
     *         outtake had been requested.
     */
    boolean outtakeHandled() {
        return m_request.compareAndSet(IntakeRequest.OUTTAKE_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
    }

    /**
     * @return true (the caller can move to next state machine state) if a
     *         retraction had been requested.
     */
    boolean retractionHandled() {
        return m_request.compareAndSet(IntakeRequest.RETRACTION_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
    }
}
