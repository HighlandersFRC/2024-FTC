package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class IntakeDefault implements Command {
    private Intake intakeSubsystem;
    public IntakeDefault(Intake subystem){
        this.intakeSubsystem = subystem;
    }
    @Override
    public void start() {

    }

    @Override
    public void execute() {
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }
}
