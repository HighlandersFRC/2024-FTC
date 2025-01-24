package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.elevatorPID;

import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;


public class ElevatorCommand implements Command {

    private double setPos;
    private double elevatorPower;
    public static double elePos = 0; // Static variable
    private String name = "Elevator";
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetPos) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.setPos = targetPos;
        System.out.println("Created Elevator with TargetPos: " + targetPos);
    }

    @Override
    public void start() {
        System.out.println(setPos);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setPosition(setPos);
    }

    @Override
    public void end() {
        elevatorSubsystem.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorSubsystem.setPower(0);
        System.out.println("Command ended.");
    }

    @Override
    public boolean isFinished() {
        double tolerance = 10;
        double currentPosition = elevatorSubsystem.getCurrentPosition();
        return Math.abs(currentPosition - setPos) <= tolerance;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevatorSubsystem;
    }
}