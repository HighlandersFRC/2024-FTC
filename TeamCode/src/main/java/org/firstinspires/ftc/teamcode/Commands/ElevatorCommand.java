package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.elevatorPID;


import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class ElevatorCommand implements Command  {

    private double setElePos;
    private double ElevatorPower;
    public static double elePos = 0;
    private String name = "Elevator";
    private ElevatorSubsystem elevator;
    public ElevatorCommand(ElevatorSubsystem elevator, double targetPos) {
        this.elevator = elevator;
        this.setElePos = targetPos;
        elevatorPID.setSetPoint(targetPos);
        elevatorPID.setMaxOutput(1);
        elevatorPID.setMinInput(-180);
        elevatorPID.setMaxInput(180);

    }
    @Override
    public void start() {

    }

    @Override
    public void execute() {
elePos = elevator.getCurrentPosition(elevator.Elevator);
ElevatorPower = elevatorPID.updatePID(elePos);
elevator.setPower(elevator.Elevator, -ElevatorPower);
    }

    @Override
    public void end() {
         elevator.setPower(elevator.Elevator, 0);
        System.out.println("Command ended.");
    }

    @Override
    public boolean isFinished() {
        double tolerance = 0;
        double currentPosition = elevator.getCurrentPosition(elevator.Elevator);
        return Math.abs(currentPosition - setElePos) <= tolerance;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevator;
    }
}
