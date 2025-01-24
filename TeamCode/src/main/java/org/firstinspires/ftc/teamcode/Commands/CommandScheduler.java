//package org.firstinspires.ftc.teamcode.Commands;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.qualcomm.robotcore.util.RobotLog;
//import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.Tools.Robot;
//
//import java.util.*;
//
//public class CommandScheduler {
//
//    Robot robot = new Robot(hardwareMap);
//
//    private static CommandScheduler instance;
//    private final List<Command> scheduledCommands = new ArrayList<>();
//    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();
//
//
//    public static CommandScheduler getInstance() {
//        if (instance == null) {
//            instance = new CommandScheduler();
//        }
//        return instance;
//    }
//
//    public void schedule(Command command) {
//        Subsystem requiredSubsystem = command.getRequiredSubsystem();
//
//        if (requiredSubsystem != null) {
//            Command activeCommand = activeSubsystemCommands.get(requiredSubsystem);
//
//            // Cancel the currently active command only if it is not a default command
//            if (activeCommand != null && !isDefaultCommand(activeCommand)) {
//                cancel(activeCommand);
//            }
//
//            // Schedule the new command
//            activeSubsystemCommands.put(requiredSubsystem, command);
//        }
//
//        command.start();
//        scheduledCommands.add(command);
//        RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
//    }
//
//    public void run() {
//        List<Command> finishedCommands = new ArrayList<>();
//
//        // Iterate through all scheduled commands and check if they have finished
//        for (Command command : new ArrayList<>(scheduledCommands)) {
//            if (command.isFinished()) {
//                command.end();
//                finishedCommands.add(command);
//                RobotLog.d("Command Finished and Ended: " + command.getClass().getSimpleName());
//
//                Subsystem subsystem = command.getRequiredSubsystem();
//                if (subsystem != null) {
//                    activeSubsystemCommands.remove(subsystem);
//
//                    // Only reschedule default command if no higher-priority command is active
//                    if (subsystem.getDefaultCommand() != null && !activeSubsystemCommands.containsKey(subsystem)) {
//                        Command defaultCommand = subsystem.getDefaultCommand();
//                        if (!scheduledCommands.contains(defaultCommand)) {
//                            RobotLog.d("Scheduling Default Command: " + defaultCommand.getClass().getSimpleName());
//                            schedule(defaultCommand);
//                        }
//                    }
//                }
//            } else {
//                command.execute();
//            }
//        }
//
//        scheduledCommands.removeAll(finishedCommands);
//
//        // Ensure that default commands are scheduled only when no other commands are active
//        for (Subsystem subsystem : getAllSubsystems()) {
//            if (!activeSubsystemCommands.containsKey(subsystem)) {
//                Command defaultCommand = subsystem.getDefaultCommand();
//                if (defaultCommand != null && !scheduledCommands.contains(defaultCommand)) {
//                    RobotLog.d("Default Command Triggered for Subsystem: " + subsystem.getClass().getSimpleName());
//                    schedule(defaultCommand);
//                }
//            }
//        }
//    }
//
//    public void printCurrentCommands() {
//        RobotLog.d("===== Current Commands =====");
//        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
//            Subsystem subsystem = entry.getKey();
//            Command command = entry.getValue();
//            RobotLog.d("Subsystem: " + subsystem.getClass().getSimpleName() +
//                    ", Command: " + command.getClass().getSimpleName());
//        }
//        RobotLog.d("============================");
//    }
//
//    private void cancel(Command command) {
//        Subsystem requiredSubsystem = command.getRequiredSubsystem();
//        if (requiredSubsystem != null) {
//            activeSubsystemCommands.remove(requiredSubsystem);
//        }
//
//        command.end();
//        scheduledCommands.remove(command);
//        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
//    }
//
//    private Set<Subsystem> getAllSubsystems() {
//        Set<Subsystem> subsystems = new HashSet<>();
//        subsystems.add(robot.intakeSubsystem);
//        subsystems.add(robot.wrist);
//        subsystems.add(robot.arm);
//        subsystems.add(robot.drive);
//        return subsystems;
//    }
//    public void setRobot(Robot robot) {
//      this.robot = robot;
//  }
//
//    private boolean isDefaultCommand(Command command) {
//        return command.getClass().getSimpleName().contains("Default");
//    }
//}

package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Robot;

import java.util.*;

public class CommandScheduler {
    private static CommandScheduler instance;
    private final List<Command> scheduledCommands = new ArrayList<>();
    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();
    private Robot robot;

    public CommandScheduler() {

    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public void schedule(Command command) {
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

    public void run() {
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

    public void printCurrentCommands() {
        RobotLog.d("===== Current Commands =====");
        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
            RobotLog.d("Subsystem: " + entry.getKey().getClass().getSimpleName() +
                    ", Command: " + entry.getValue().getClass().getSimpleName());
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
        if (robot != null) {
            subsystems.add(robot.arm);
            subsystems.add(robot.drive);
            subsystems.add(robot.intakeSubsystem);
            subsystems.add(robot.wrist);
        }
        return subsystems;
    }

    public boolean isCommandScheduled(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && activeSubsystemCommands.get(subsystem) == command;
    }

    private boolean isDefaultCommand(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && subsystem.getDefaultCommand() == command;
    }
}
