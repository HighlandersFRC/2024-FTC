package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.List;

public class CommandScheduler {
    private List<Command> commandList;
    private static CommandScheduler instance;

    public CommandScheduler() {
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
        command.start(); // Ensure command start is called when scheduled
    }

    public void schedule(Command... commands) {
        for (Command command : commands) {
            schedule(command); // Use the single command schedule to ensure start is called
        }
    }

    public void schedule(List<Command> commands) {
        for (Command command : commands) {
            schedule(command); // Use the single command schedule to ensure start is called
        }
    }

    public void cancel(Command command) {
        if (commandList.contains(command)) {
            command.end();
            commandList.remove(command);
        }
    }

    public void cancelAll() {
        for (Command command : commandList) {
            command.end();
        }
        commandList.clear();
    }

    public void run() throws InterruptedException {
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
