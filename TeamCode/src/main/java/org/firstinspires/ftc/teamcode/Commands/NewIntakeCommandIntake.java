package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class NewIntakeCommandIntake implements Command{
    NewIntakeSubsystem intakeSubsystem;

    public NewIntakeCommandIntake(NewIntakeSubsystem intake) {
        this.intakeSubsystem = intake;

    }

    @Override
    public void start() {
        intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.INTAKE);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.DEFAULT);
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
