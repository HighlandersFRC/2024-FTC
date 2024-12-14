package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class Elevator implements Command{
    private final PID elevatorPID = new PID(0.005, 0.004, 0.01);

    String name = "Elevator";

    public Elevator(double pos){
        elevatorPID.setSetPoint(pos);
    }

    @Override
    public void start() {
        RobotLog.d("Starting Elevator Command");
    }

    @Override
    public void execute() {
        Pivot.setPower(1);
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
        if (Math.abs(elevatorPID.getSetPoint() - averageEncoderPosition) < 50) {
            return true;
        }
        return false;
    }
}
