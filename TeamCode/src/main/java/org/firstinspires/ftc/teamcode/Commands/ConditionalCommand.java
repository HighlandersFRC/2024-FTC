package org.firstinspires.ftc.teamcode.Commands;


public class ConditionalCommand implements Command {

    private Command onTrue;
    private Command onFalse;
    private Command CommandToRun;
    private boolean condition;

    public ConditionalCommand(Boolean Condition,Command onTrue, Command onFalse) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = Condition;
    }

    @Override
    public void start() {
        if (condition){
            CommandToRun = onTrue;
        }
        else {
            CommandToRun = onFalse;
        }
    }



    @Override
    public void execute() {
        CommandToRun.execute();;
    }

    @Override
    public void end() {
        CommandToRun.end();

    }

    @Override
    public boolean isFinished() {
        return CommandToRun.isFinished();
    }

    @Override
    public String getSubsystem() {
        return CommandToRun.getSubsystem();
    }
}