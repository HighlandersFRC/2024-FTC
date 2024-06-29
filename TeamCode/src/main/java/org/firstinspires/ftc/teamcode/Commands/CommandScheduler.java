package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.List;

public class CommandScheduler {

    private List<Command> commandList;
    private static CommandScheduler instance;

    private CommandScheduler() {
        commandList = new ArrayList<>();
    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public void schedule(Command command) {
        commandList.add(command);
    }

    public void cancel(Command command) {
        if (commandList.contains(command)) {
            commandList.remove(command);
            command.end();
        }
    }

    public void cancelAll() {
        for (Command command : commandList) {
            command.end();
        }
        commandList.clear();
    }

    public void run() {
        List<Command> finishedCommands = new ArrayList<>();
        for (Command command : commandList) {
            if (!command.isFinished()) {
                command.execute();
            } else {
                command.end();
                finishedCommands.add(command);
            }
        }
        commandList.removeAll(finishedCommands);
    }

    public boolean isScheduled(Command command) {
        return commandList.contains(command);
    }

    public boolean isScheduled() {
        return !commandList.isEmpty();
    }

    public int size() {
        return commandList.size();
    }

    public void clear() {
        commandList.clear();
    }

    public void remove(Command command) {
        commandList.remove(command);
    }

    public void remove(int index) {
        commandList.remove(index);
    }
}
