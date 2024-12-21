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
            if (activeCommand != null && !isDefaultCommand(activeCommand)) {
                cancel(activeCommand);
            }
            activeSubsystemCommands.put(requiredSubsystem, command);
        }

        command.start();
        scheduledCommands.add(command);
        RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
    }

    public void run() {
        List<Command> finishedCommands = new ArrayList<>();

        for (Command command : new ArrayList<>(scheduledCommands)) {
            if (command.isFinished()) {
                command.end();
                finishedCommands.add(command);
                RobotLog.d("Command Finished and Ended: " + command.getClass().getSimpleName());
            } else {
                command.execute();
            }
        }

        scheduledCommands.removeAll(finishedCommands);

        for (Subsystem subsystem : getAllSubsystems()) {
            if (!activeSubsystemCommands.containsKey(subsystem)) {
                Command defaultCommand = subsystem.getDefaultCommand();
                if (defaultCommand != null && !scheduledCommands.contains(defaultCommand) && !isDefaultCommand(defaultCommand)) {
                    schedule(defaultCommand);
                }
            }
        }
    }

    public void cancel(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            activeSubsystemCommands.remove(requiredSubsystem);
        }

        command.end();
        scheduledCommands.remove(command);
        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
    }

    public void cancelAll() {
        for (Command command : new ArrayList<>(scheduledCommands)) {
            cancel(command);
        }
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
