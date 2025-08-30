package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewElevatorCommandRetract implements Command{
    Superstructure superstructure;

    public NewElevatorCommandRetract(Superstructure superstructure) {
        this.superstructure = superstructure;

    }

    @Override
    public void start() {
        superstructure.setWantedState(Superstructure.SUPER_STATE.ELEVATOR_RETRACT);
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
