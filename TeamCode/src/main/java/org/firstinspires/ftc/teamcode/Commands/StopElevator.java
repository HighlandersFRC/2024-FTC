package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class StopElevator implements Command{
    private ElevatorSubsystem elevator;
    @Override
    public void start() {
        System.out.println("Elevator Stop Started");
    }

    @Override
    public void execute() {
elevator.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setPower(0);
    }

    @Override
    public void end() {
        elevator.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevator;
    }
}
