package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class Elevators implements Command{
    private final PID elevatorPID = new PID(0.003, 0.0, 0.00);

    public Elevators(){
        elevatorPID.setSetPoint(Robot.CURRENT_ELEVATOR);
    }

    @Override
    public String getSubsystem() {
        return "";
    }

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        org.firstinspires.ftc.teamcode.Subsystems.Elevators.moveLeftElevator(elevatorPID.updatePID(org.firstinspires.ftc.teamcode.Subsystems.Elevators.getLeftEncoder()));
        org.firstinspires.ftc.teamcode.Subsystems.Elevators.moveRightElevator(elevatorPID.updatePID(org.firstinspires.ftc.teamcode.Subsystems.Elevators.getLeftEncoder()));
    }

    @Override
    public void end() {
        org.firstinspires.ftc.teamcode.Subsystems.Elevators.moveLeftElevator(0);
        org.firstinspires.ftc.teamcode.Subsystems.Elevators.moveRightElevator(0);
        org.firstinspires.ftc.teamcode.Subsystems.Elevators.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        double averageEncoderPosition = (org.firstinspires.ftc.teamcode.Subsystems.Elevators.getLeftEncoder() + org.firstinspires.ftc.teamcode.Subsystems.Elevators.getRightEncoder()) / 2;
        if (Math.abs(elevatorPID.getSetPoint() - averageEncoderPosition) < 10) {
            return true;
        }
        return false;
    }
}