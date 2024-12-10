package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Elevators;

public class Elevator implements Command {
    private final boolean moveUp;

    String name = "Elevator";


    public Elevator(boolean moveUp) {
        this.moveUp = moveUp;
    }

    @Override
    public String getSubsystem() {
        return "";
    }

    @Override
    public void start() {
        if (moveUp) {
            Elevators.moveUp();
        } else {
            Elevators.moveDown();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        Elevators.stop();
        Elevators.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
