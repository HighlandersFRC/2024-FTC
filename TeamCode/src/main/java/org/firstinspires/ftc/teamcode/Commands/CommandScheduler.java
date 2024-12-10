package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.json.JSONException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CommandScheduler {
    private static CommandScheduler instance;
    private static List<Command> scheduledCommands = new ArrayList<>();

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

}