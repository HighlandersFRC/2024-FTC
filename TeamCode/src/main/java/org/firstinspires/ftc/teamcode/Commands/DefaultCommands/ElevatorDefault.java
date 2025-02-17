package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import static org.firstinspires.ftc.teamcode.Tools.Robot.CURRENT_STATE;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class ElevatorDefault implements Command {

    private final PID elevatorPID = new PID(0.005, 0.004, 0.01); // PID for elevator control
    private final Elevators elevators = Robot.elevators;

    private static final double LOWER_LIMIT = -2200;
    private static final double UPPER_LIMIT = 2300;
    private static final double PIVOT_THRESHOLD = 10; // Threshold for pivot to switch control mode

    String name = "Elevator";

    @Override
    public void start() {
        RobotLog.d("Starting Elevator Command");
        elevatorPID.setSetPoint(Robot.CURRENT_ELEVATOR); // Set initial elevator position for PID control
    }

    @Override
    public void execute() {
        if (CURRENT_STATE.equals("Auto")){
            Constants.PIVOT_STARTING_ANGLE  = 16.9;
        }
        else {
            Constants.PIVOT_STARTING_ANGLE = -16.1;
        }
        // Only apply the changes during TeleOp mode
        if (CURRENT_STATE.equals("Tele-Op")) {

            Constants.PIVOT_STARTING_ANGLE = -16.1;


            double power = Robot.elevatorPower;
            double leftEncoder = elevators.getLeftEncoder();
            double rightEncoder = elevators.getRightEncoder();
            double pivotPosition = Robot.CURRENT_PIVOT;

            if (pivotPosition < PIVOT_THRESHOLD) {
                // If pivot position is lower than threshold, manual control (no retract, holds position)
                if (power == 0) {
                    elevators.moveLeftElevator(0);
                    elevators.moveRightElevator(0);
                } else if (power > 0 && leftEncoder < UPPER_LIMIT && rightEncoder < UPPER_LIMIT) {
                    elevators.moveLeftElevator(1);
                    elevators.moveRightElevator(1);
                } else if (power < 0 && leftEncoder > LOWER_LIMIT && rightEncoder > LOWER_LIMIT) {
                    elevators.moveLeftElevator(-1);
                    elevators.moveRightElevator(-1);
                } else {
                    elevators.moveLeftElevator(0);
                    elevators.moveRightElevator(0);
                }
            } else {
                // If pivot position is higher than threshold, PID control
                if (power == 0) {
                    elevators.moveLeftElevator(elevatorPID.updatePID(leftEncoder));
                    elevators.moveRightElevator(elevatorPID.updatePID(rightEncoder));
                } else {
                    elevatorPID.setSetPoint(Robot.CURRENT_ELEVATOR);
                    if (power > 0 && leftEncoder < UPPER_LIMIT && rightEncoder < UPPER_LIMIT) {
                        elevators.moveLeftElevator(1);
                        elevators.moveRightElevator(1);
                    } else if (power < 0 && leftEncoder > LOWER_LIMIT && rightEncoder > LOWER_LIMIT) {
                        elevators.moveLeftElevator(-1);
                        elevators.moveRightElevator(-1);
                    } else {
                        elevators.moveLeftElevator(0);
                        elevators.moveRightElevator(0);
                    }
                }
            }
        } else if (CURRENT_STATE.equals("Auto")) {

        }
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return elevators;
    }
}
