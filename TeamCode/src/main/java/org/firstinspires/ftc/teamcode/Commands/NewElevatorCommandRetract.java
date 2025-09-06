package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewElevatorCommandRetract implements Command{
    NewElevatorSubsystem elevatorSubsystem;

    public NewElevatorCommandRetract(NewElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

    }

    @Override
    public void start() {
        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_RETRACT);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevatorSubsystem;
    }
}
