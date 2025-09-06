package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewIntakeCommandOuttake implements Command{
    NewIntakeSubsystem intakeSubsystem;
    Superstructure superstructure;
    public NewIntakeCommandOuttake(NewIntakeSubsystem intake, Superstructure superstructure) {
        this.intakeSubsystem = intake;
        this.superstructure = superstructure;
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
