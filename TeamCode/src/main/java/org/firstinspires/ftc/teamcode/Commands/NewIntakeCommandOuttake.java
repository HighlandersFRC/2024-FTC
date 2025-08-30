package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewIntakeCommandOuttake implements Command{
    Superstructure superstructure;

    public NewIntakeCommandOuttake(Superstructure superstructure) {
        this.superstructure = superstructure;

    }

    @Override
    public void start() {
        superstructure.setWantedState(Superstructure.SUPER_STATE.OUTTAKE);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        superstructure.setWantedState(Superstructure.SUPER_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return superstructure;
    }
}
