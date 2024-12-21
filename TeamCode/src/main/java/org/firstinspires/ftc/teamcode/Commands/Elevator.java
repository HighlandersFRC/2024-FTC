package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class Elevator implements Command {

    private final PID elevatorPID = new PID(0.005, 0.004, 0.01);
    private final Elevators elevators;
    private final double targetPosition;

    String name = "Elevator";

    public Elevator(Elevators elevators, double pos) {
        this.elevators = elevators;
        this.targetPosition = pos;
        elevatorPID.setSetPoint(pos);
    }

    @Override
    public void start() {
        RobotLog.d("Starting Elevator Command");
    }

    @Override
    public void execute() {
        elevators.moveLeftElevator(elevatorPID.updatePID(elevators.getLeftEncoder()));
        elevators.moveRightElevator(elevatorPID.updatePID(elevators.getLeftEncoder()));
    }

    @Override
    public void end() {
        elevators.moveLeftElevator(0);
        elevators.moveRightElevator(0);
        elevators.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        double averageEncoderPosition = (elevators.getLeftEncoder() + elevators.getRightEncoder()) / 2;
        return Math.abs(elevatorPID.getSetPoint() - averageEncoderPosition) < 50;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevators; // Return the instance of the subsystem
    }
}
