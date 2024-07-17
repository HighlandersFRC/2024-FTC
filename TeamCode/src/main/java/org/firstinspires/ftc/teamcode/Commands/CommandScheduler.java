package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class CommandScheduler {
    private List<Command> commandList = new ArrayList<>();

    private static CommandScheduler instance;

    private CommandScheduler() {
        // Private constructor to prevent instantiation
    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public Command getActive() {
        if (!commandList.isEmpty()) {
            return commandList.get(0);
        }
        return null;
    }

    public void schedule(Command command) {
        commandList.add(command);
        command.start();
    }

    public void run() throws InterruptedException {
        Iterator<Command> iterator = commandList.iterator();
        while (iterator.hasNext()) {
            Command command = iterator.next();
            if (command.isFinished()) {
                command.end();
                iterator.remove();
            } else {
                command.execute();
            }
        }
    }

    public void cancel(Command command) {
        command.end();
        commandList.remove(command);
    }

    public void cancelAll() {
        for (Command command : commandList) {
            command.end();
        }
        commandList.clear();
    }

    public boolean isScheduled(Command command) {
        return commandList.contains(command);
    }
}
