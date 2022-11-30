// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.IdentityHashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RumbleCommand;

/**
 * It is desirable to have a single rumble command instance per controller so
 * that if a command tries to rumble while we are already rumbling (schedule the
 * rumble command while it is already running), the second rumble is ignored.
 * The proper way to compose commands in WPILIB (any given commmand and a rumble
 * in this case) is to fold them into a command group, usually via one of the
 * decorator methods on the {@link Command} class (for example,
 * {@link Command#andThen(Command...)}). The problem with this approach in our
 * case is that a single command instance can only be included in one command
 * group. Using andThen directly would require a new rumble commmand instance
 * for each group. That is where this factory and its usage of
 * {@link ScheduleCommand} comes in. The schedule command is an instant command
 * that simply schedules the commands it is given. The schedule command will
 * belong to the command group (therefore a new instance each time) but its
 * forked commands, in this case just our single rumble command, will not.
 */
public class RumbleFactory {
    /** The single instance of the factory. */
    private static final RumbleFactory INSTANCE = new RumbleFactory();

    /** Map of a controller to its {@link RumbleCommand}. */
    private final Map<GenericHID, Command> m_commandMap = new IdentityHashMap<>();

    /** Private to ensure only a single instance is created. */
    private RumbleFactory() {
    }

    /**
     * @return the single {@link RumbleFactory} instance.
     */
    public static RumbleFactory getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new {@link RumbleCommand} the first time it is requested for the
     * generic HID and returns that same instance on subsequent calls for the same
     * generic HID.
     */
    private Command getCommand(final GenericHID genericHID) {
        return m_commandMap.computeIfAbsent(
                genericHID,
                gh -> new RumbleCommand(gh)
                        .raceWith(new WaitCommand(1.0))); // Rumble for 1 second.
    }

    /**
     * The factory method.
     * 
     * @param genericHID the generic human interaction device (HID), aka controller,
     *                   to be rumbled.
     * 
     * @return a new {@link ScheduleCommand} which will schedule the HID's singleton
     *         {@link RumbleCommand}.
     */
    public ScheduleCommand getGroupableCommand(final GenericHID genericHID) {
        return new ScheduleCommand(getCommand(genericHID));
    }
}
