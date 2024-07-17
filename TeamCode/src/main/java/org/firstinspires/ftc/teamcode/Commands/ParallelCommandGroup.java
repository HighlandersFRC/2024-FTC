package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Tools.Parameters;

import java.util.ArrayList;
import java.util.List;

public class ParallelCommandGroup implements Command {
    private List<Command> commands = new ArrayList<>();
    private Parameters parameters;

    public ParallelCommandGroup(Parameters parameters, Command... commands) {
        this.parameters = parameters;
        for (Command command : commands) {
            this.commands.add(command);
        }
    }

    @Override
    public void start() {
        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public void execute() throws InterruptedException {
        for (Command command : commands) {
            command.execute();
        }
    }

    @Override
    public void end() {
        for (Command command : commands) {
            command.end();
        }
    }

    @Override
    public boolean isFinished() {
        if (parameters == Parameters.ALL) {
            for (Command command : commands) {
                if (!command.isFinished()) {
                    return false;
                }
            }
            return true;
        } else if (parameters == Parameters.ANY) {
            for (Command command : commands) {
                if (command.isFinished()) {
                    return true;
                }
            }
            return false;
        } else { // Parameters.SPECIFIC
            // Implement specific logic if needed
            return true; // Placeholder
        }
    }

    @Override
    public String getSubsystem() {
        // Return a combined subsystem string if needed
        return "Combined subsystems"; // Placeholder
    }
}
