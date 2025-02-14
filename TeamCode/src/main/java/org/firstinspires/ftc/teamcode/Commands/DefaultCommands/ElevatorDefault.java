package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;


import static org.firstinspires.ftc.teamcode.Commands.ElevatorCommand.elePos;
import static org.firstinspires.ftc.teamcode.Tools.Constants.GravityTerm;
import static org.firstinspires.ftc.teamcode.Tools.Constants.elevatorPID;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class ElevatorDefault implements Command{

    private static final double gravityEffect = 1.47;
    private double elevatorPower;

    private String name = "Elevator";

    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorDefault(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }
    @Override
    public void start() {
elevatorPID.setSetPoint(-1 * Math.abs(elePos));
        System.out.println("ElevatorDefault started");
    }

    @Override
    public void execute() {
elevatorPower = elevatorPID.updatePIDF(elevatorSubsystem.getCurrentPosition(), gravityEffect);
double feed = GravityTerm(elevatorSubsystem.getCurrentPosition());
elevatorSubsystem.setPower(-elevatorPower * feed);
    }

    @Override
    public void end() {
elevatorSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        double tolerance = 0;
        double currentPosition = elevatorSubsystem.getCurrentPosition();
        return Math.abs(currentPosition - elePos) <= tolerance;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevatorSubsystem;
    }
}
