package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewIntakeCommandOuttake implements Command{
    NewIntakeSubsystem intakeSubsystem;

    public NewIntakeCommandOuttake(NewIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

    }

    @Override
    public void start() {
        intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.OUTTAKE);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.OUTTAKE);
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
