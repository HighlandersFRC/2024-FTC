package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class Elevator implements Command{
    private final PID elevatorPID = new PID(0.007, 0.0, 0.008);

    public Elevator(){
        elevatorPID.setSetPoint(Robot.CURRENT_ELEVATOR);
    }

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        Elevators.moveLeftElevator(elevatorPID.updatePID(Elevators.getLeftEncoder()));
        Elevators.moveRightElevator(elevatorPID.updatePID(Elevators.getLeftEncoder()));
    }

    @Override
    public void end() {
        Elevators.moveLeftElevator(0);
        Elevators.moveRightElevator(0);
        Elevators.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        double averageEncoderPosition = (Elevators.getLeftEncoder() + Elevators.getRightEncoder()) / 2;
        if (Math.abs(elevatorPID.getSetPoint() - averageEncoderPosition) < 10) {
            return true;
        }
        return false;
    }
}
