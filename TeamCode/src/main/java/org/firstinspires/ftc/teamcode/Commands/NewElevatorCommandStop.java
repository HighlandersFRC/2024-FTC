package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewElevatorCommandStop implements Command{
    NewElevatorSubsystem elevatorSubsystem;

    public NewElevatorCommandStop(NewElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

    }

    @Override
    public void start() {
        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.DEFAULT);
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
