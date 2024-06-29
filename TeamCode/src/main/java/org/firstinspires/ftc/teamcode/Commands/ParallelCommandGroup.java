package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.List;

public class ParallelCommandGroup implements Command {

    private List<Command> commands;
    private boolean isFinished;

    public ParallelCommandGroup() {
        commands = new ArrayList<>();
        isFinished = false;
    }

    public void addCommand(Command command) {
        commands.add(command);
    }


    public void start() {
        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public void execute() {
        for (Command command : commands) {
            if (!command.isFinished()) {
                command.execute();
            }
        }
        checkCompletion(); // Check if all commands are finished
    }

    private void checkCompletion() {
        boolean allFinished = true;
        for (Command command : commands) {
            if (!command.isFinished()) {
                allFinished = false;
                break;
            }
        }
        isFinished = allFinished;
    }

    @Override
    public void end() {
        for (Command command : commands) {
            command.end();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public String getSubsystem() {
        // Return subsystem of the first command (assuming all use the same subsystem)
        if (!commands.isEmpty()) {
            return commands.get(0).getSubsystem();
        }
        return "";
    }
}
