package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class Elevator implements Command {

    private final PID elevatorPID = new PID(0.3, 0.0, 0.0);
    private final Elevators elevators;
    private final double targetPosition;

    String name = "Elevator";

    public Elevator(Elevators elevators, double pos) {
        this.elevators = elevators;
        this.targetPosition = pos;
        Robot.CURRENT_ELEVATOR = pos;
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

        Robot.CURRENT_ELEVATOR = (Elevators.getLeftEncoder() + Elevators.getRightEncoder()) / 2;
    }

    @Override
    public boolean isFinished() {
        double averageEncoderPosition = (elevators.getLeftEncoder() + elevators.getRightEncoder()) / 2;
        if (Math.abs(elevatorPID.getSetPoint() - averageEncoderPosition) < 200 || averageEncoderPosition < -20){
            if (averageEncoderPosition < -30){
                Elevators.resetEncoders();
                return true;
            }
            return true;
        }
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevators;
    }
}
