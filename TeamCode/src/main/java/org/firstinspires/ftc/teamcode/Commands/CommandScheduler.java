package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Robot;

import java.util.*;

public class CommandScheduler {
    private static CommandScheduler instance;
    private final List<Command> scheduledCommands = new ArrayList<>();
    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public void schedule(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();

        if (requiredSubsystem != null) {
            Command activeCommand = activeSubsystemCommands.get(requiredSubsystem);

            // Cancel the currently active command only if it is not a default command
            if (activeCommand != null && !isDefaultCommand(activeCommand)) {
                cancel(activeCommand);
            }

            // Schedule the new command
            activeSubsystemCommands.put(requiredSubsystem, command);
        }

        command.start();
        scheduledCommands.add(command);
        RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
    }

    public void run() {
        Robot.run();
        List<Command> finishedCommands = new ArrayList<>();

        // Iterate through all scheduled commands and check if they have finished
        for (Command command : new ArrayList<>(scheduledCommands)) {
            if (command.isFinished()) {
                command.end();
                finishedCommands.add(command);
                RobotLog.d("Command Finished and Ended: " + command.getClass().getSimpleName());

                Subsystem subsystem = command.getRequiredSubsystem();
                if (subsystem != null) {
                    activeSubsystemCommands.remove(subsystem);

                    // Only reschedule default command if no higher-priority command is active
                    if (subsystem.getDefaultCommand() != null && !activeSubsystemCommands.containsKey(subsystem)) {
                        Command defaultCommand = subsystem.getDefaultCommand();
                        if (!scheduledCommands.contains(defaultCommand)) {
                            RobotLog.d("Scheduling Default Command: " + defaultCommand.getClass().getSimpleName());
                            schedule(defaultCommand);
                        }
                    }
                }
            } else {
                command.execute();
            }
        }

        scheduledCommands.removeAll(finishedCommands);

        // Ensure that default commands are scheduled only when no other commands are active
        for (Subsystem subsystem : getAllSubsystems()) {
            if (!activeSubsystemCommands.containsKey(subsystem)) {
                Command defaultCommand = subsystem.getDefaultCommand();
                if (defaultCommand != null && !scheduledCommands.contains(defaultCommand)) {
                    RobotLog.d("Default Command Triggered for Subsystem: " + subsystem.getClass().getSimpleName());
                    schedule(defaultCommand);
                }
            }
        }
    }

    public void printCurrentCommands() {
        RobotLog.d("===== <Current Commands> =====");
        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
            Subsystem subsystem = entry.getKey();
            Command command = entry.getValue();
            RobotLog.d("Subsystem: " + subsystem.getClass().getSimpleName() +
                    ", Command: " + command.getClass().getSimpleName());
        }
        RobotLog.d("============================");
    }

    private void cancel(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            activeSubsystemCommands.remove(requiredSubsystem);
        }

        command.end();
        scheduledCommands.remove(command);
        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
    }

    private Set<Subsystem> getAllSubsystems() {
        Set<Subsystem> subsystems = new HashSet<>();
        subsystems.add(Robot.elevators);
        subsystems.add(Robot.pivot);
        subsystems.add(Robot.intake);
        subsystems.add(Robot.wrist);
        return subsystems;
    }

    private boolean isDefaultCommand(Command command) {
        return command.getClass().getSimpleName().contains("Default");
    }
}
