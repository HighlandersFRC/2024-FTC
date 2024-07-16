package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SequentialCommandGroup implements Command {
    private List<Command> commands;
    private int currentCommandIndex;
    private boolean isFinished;

    public SequentialCommandGroup(Command... commands) {
        this.commands = new ArrayList<>();
        Collections.addAll(this.commands, commands);
        currentCommandIndex = 0;
        isFinished = false;
    }

    @Override
    public void start() {
        if (!commands.isEmpty()) {
            commands.get(currentCommandIndex).start();
        }
    }

    @Override
    public void execute() throws InterruptedException {
        if (!commands.isEmpty() && currentCommandIndex < commands.size()) {
            Command currentCommand = commands.get(currentCommandIndex);
            if (!currentCommand.isFinished()) {
                currentCommand.execute();
            }
            if (currentCommand.isFinished()) {
                currentCommand.end();
                currentCommandIndex++;
                if (currentCommandIndex < commands.size()) {
                    commands.get(currentCommandIndex).start();
                } else {
                    isFinished = true;
                }
            }
        } else {
            isFinished = true;
        }
    }

    @Override
    public void end() {
        if (!commands.isEmpty() && currentCommandIndex < commands.size()) {
            commands.get(currentCommandIndex).end();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public String getSubsystem() {
        if (!commands.isEmpty() && currentCommandIndex < commands.size()) {
            return commands.get(currentCommandIndex).getSubsystem();
        }
        return "";
    }
}
