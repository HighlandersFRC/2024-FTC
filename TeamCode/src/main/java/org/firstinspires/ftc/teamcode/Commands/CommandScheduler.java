package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.json.JSONException;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CommandScheduler {
    private static CommandScheduler instance;
    private static List<Command> scheduledCommands = new ArrayList<>();
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        return o != null && getClass() == o.getClass();
    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public static void add(CommandScheduler scheduler, Command... commands) {
        scheduledCommands.addAll(Arrays.asList(commands));
    }

    public void schedule(Command command) {
        if (!scheduledCommands.contains(command)) { // Check for duplicates
            command.start();
            scheduledCommands.add(command);
            RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
        } else {
            RobotLog.d("Command Already Scheduled: " + command.getClass().getSimpleName());
        }
    }


    public void run() throws InterruptedException, JSONException {
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
    }

    public void cancel(Command command) {
        command.end();
        scheduledCommands.remove(command);
        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
    }

    public void cancelAll() {
        for (Command command : new ArrayList<>(scheduledCommands)) {
            command.end();
            RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
        }
        scheduledCommands.clear();
    }

    public void overrideSpecificCommand(Command newCommand, Class<? extends Command> targetCommandClass) {
        Command toCancel = null;

        for (Command command : scheduledCommands) {
            if (targetCommandClass.isInstance(command)) {
                if (command.getClass().equals(newCommand.getClass())) {
                    RobotLog.d("No override performed; the same class is already scheduled: " + command.getClass().getSimpleName());
                    return;
                }
                toCancel = command;
                break;
            }
        }

        if (toCancel != null) {
            toCancel.end();
            scheduledCommands.remove(toCancel);
            RobotLog.d("Command Cancelled: " + toCancel.getClass().getSimpleName());
        }

        schedule(newCommand);
        RobotLog.d("Command Overridden with: " + newCommand.getClass().getSimpleName());
    }

    public void RunAfterSpecificCommandIsFinished(Command newCommand, Class<? extends Command> targetCommandClass){
        Command IsFinished = null;

        for (Command command : scheduledCommands) {
            if (targetCommandClass.isInstance(command)) {
                IsFinished = command;
                break;
            }
        }

        if(IsFinished.isFinished()){
            schedule(newCommand);
        }
    }

    public void removeDuplicateCommands() {
        List<Command> uniqueCommands = new ArrayList<>();

        for (Command command : new ArrayList<>(scheduledCommands)) {
            String name = command.getClass().getSimpleName();
            scheduledCommands.removeIf(c -> c.getClass().getSimpleName().equalsIgnoreCase(name));
            uniqueCommands.add(command);
        }

        scheduledCommands.addAll(uniqueCommands);
    }


    private boolean compareCommandProperties(Command command1, Command command2) {
        if (command1.getClass().equals(command2.getClass())) {
            try {
                for (Field field : command1.getClass().getDeclaredFields()) {
                    field.setAccessible(true);
                    Object value1 = field.get(command1);
                    Object value2 = field.get(command2);

                    // Compare values, considering null values and object equality
                    if (value1 != null && value2 != null) {
                        if (!value1.equals(value2)) {
                            return false;
                        }
                    } else if (value1 != value2) {
                        return false;
                    }
                }
                return true;
            } catch (IllegalAccessException e) {
                e.printStackTrace();
                // Consider logging an error or taking other appropriate actions
                return false;
            }
        }
        return false;
    }

}