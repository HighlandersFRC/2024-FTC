package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Robot;

import java.util.*;

public class CommandScheduler {
    private static final List<Command> scheduledCommands = new ArrayList<>();
    private static final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();
    private static Robot robot;

    private CommandScheduler() {
        // Private constructor to prevent instantiation
    }

    public static void setRobot(Robot robot) {
        CommandScheduler.robot = robot;
    }

    public static void schedule(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();

        if (requiredSubsystem != null) {
            Command activeCommand = activeSubsystemCommands.get(requiredSubsystem);

            // Prevent duplicate scheduling of the same command
            if (activeCommand == command) {
                RobotLog.d("Command already active, not rescheduling: " + command.getClass().getSimpleName());
                return;
            }

            // Cancel the currently active command if it's not a default command
            if (activeCommand != null && !isDefaultCommand(activeCommand)) {
                cancel(activeCommand);
            }

            // Associate the new command with the subsystem
            activeSubsystemCommands.put(requiredSubsystem, command);
        }

        // Schedule and start the new command if not already in the list
        if (!scheduledCommands.contains(command)) {
            scheduledCommands.add(command);
            command.start();
            RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
        }
    }

    public static void run() {
        List<Command> finishedCommands = new ArrayList<>();

        // Execute scheduled commands and handle completion
        for (Command command : new ArrayList<>(scheduledCommands)) {
            if (command.isFinished()) {
                command.end();
                finishedCommands.add(command);
                RobotLog.d("Command Finished and Ended: " + command.getClass().getSimpleName());

                Subsystem subsystem = command.getRequiredSubsystem();
                if (subsystem != null) {
                    activeSubsystemCommands.remove(subsystem);

                    // Reschedule default command if no other commands are active for this subsystem
                    Command defaultCommand = subsystem.getDefaultCommand();
                    if (defaultCommand != null && !isCommandScheduled(defaultCommand)) {
                        schedule(defaultCommand);
                    }
                }
            } else {
                command.execute();
            }
        }

        scheduledCommands.removeAll(finishedCommands);

        // Ensure default commands are scheduled when needed
        for (Subsystem subsystem : getAllSubsystems()) {
            if (!activeSubsystemCommands.containsKey(subsystem)) {
                Command defaultCommand = subsystem.getDefaultCommand();
                if (defaultCommand != null && !isCommandScheduled(defaultCommand)) {
                    schedule(defaultCommand);
                }
            }
        }
    }

    public static void printCurrentCommands() {
        RobotLog.d("===== Current Commands =====");
        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
            RobotLog.d("Subsystem: " + entry.getKey().getClass().getSimpleName() +
                    ", Command: " + entry.getValue().getClass().getSimpleName());
        }
        RobotLog.d("============================");
    }

    private static void cancel(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            activeSubsystemCommands.remove(requiredSubsystem);
        }

        command.end();
        scheduledCommands.remove(command);
        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
    }

    private static Set<Subsystem> getAllSubsystems() {
        Set<Subsystem> subsystems = new HashSet<>();
        if (robot != null) {
            subsystems.add(robot.arm);
            subsystems.add(robot.drive);
            subsystems.add(robot.intake);
            subsystems.add(robot.wrist);
        }
        return subsystems;
    }

    public static boolean isCommandScheduled(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && activeSubsystemCommands.get(subsystem) == command;
    }

    private static boolean isDefaultCommand(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && subsystem.getDefaultCommand() == command;
    }
}
