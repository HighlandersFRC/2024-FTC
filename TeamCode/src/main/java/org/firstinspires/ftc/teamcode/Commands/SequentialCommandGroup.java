package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.List;

public class SequentialCommandGroup implements Command {

    private List<Command> commands;
    private int currentCommandIndex;
    private boolean isFinished;

    public SequentialCommandGroup() {

        commands = new ArrayList<>();
        currentCommandIndex = 0;
        isFinished = false;

    }

    public void addCommand(Command command) {
        commands.add(command);
    }


    public void start() {
        if (!commands.isEmpty()) {
            commands.get(currentCommandIndex).start();
        }
    }

    @Override
    public void execute() {
        if (!commands.isEmpty()) {
            commands.get(currentCommandIndex).execute();
            if (commands.get(currentCommandIndex).isFinished()) {
                commands.get(currentCommandIndex).end();
                currentCommandIndex++;
                if (currentCommandIndex >= commands.size()) {
                    isFinished = true;
                } else {
                    commands.get(currentCommandIndex).start();
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
