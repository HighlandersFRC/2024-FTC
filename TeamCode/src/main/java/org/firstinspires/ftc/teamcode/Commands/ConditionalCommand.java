package org.firstinspires.ftc.teamcode.Commands;


public class ConditionalCommand implements Command {

    private Command onTrue;
    private Command onFalse;
    private boolean conditionEvaluated;

    public ConditionalCommand(Command onTrue, Command onFalse) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.conditionEvaluated = false;
    }

    @Override
    public void start() {
        // Evaluate condition or setup
        // Example: Implement condition logic here
        boolean condition = true; // Example condition, replace with your logic
        conditionEvaluated = condition;
        Command commandToExecute = condition ? onTrue : onFalse;
        commandToExecute.start();
    }



    @Override
    public void execute() {
        // Execute appropriate command based on condition
        Command commandToExecute = conditionEvaluated ? onTrue : onFalse;
        commandToExecute.execute();
    }

    @Override
    public void end() {
        // End currently active command
        Command commandToTerminate = conditionEvaluated ? onTrue : onFalse;
        commandToTerminate.end();
    }

    @Override
    public boolean isFinished() {
        // Check if the active command is finished
        Command commandToCheck = conditionEvaluated ? onTrue : onFalse;
        return commandToCheck.isFinished();
    }

    @Override
    public String getSubsystem() {
        // Return subsystem associated with the active command
        Command commandToReturn = conditionEvaluated ? onTrue : onFalse;
        return commandToReturn.getSubsystem();
    }
}
