package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import static org.firstinspires.ftc.teamcode.Tools.Robot.elevators;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.ElevatorWithPower;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class ElevatorDefault implements Command {

    private final PID elevatorPID = new PID(0.005, 0.004, 0.01);
    private final Elevators elevators = Robot.elevators;
    String name = "Elevator";

    @Override
    public void start() {
        RobotLog.d("Starting Elevator Command");
        elevatorPID.setSetPoint(Robot.CURRENT_ELEVATOR);
    }

    @Override
    public void execute() {
        if (Robot.CURRENT_STATE.equals("Tele-Op")) {
            double power = Robot.elevatorPower;
            if ((Robot.elevatorPower == 0)) {
                elevators.moveLeftElevator(elevatorPID.updatePID(elevators.getLeftEncoder()));
                elevators.moveRightElevator(elevatorPID.updatePID(elevators.getLeftEncoder()));
            } else {
                elevatorPID.setSetPoint(Robot.CURRENT_ELEVATOR);
                if (power > 0) {
                    elevators.moveLeftElevator(1);
                    elevators.moveRightElevator(1);
                } else if (power < 0) {
                    elevators.moveLeftElevator(-1);
                    elevators.moveRightElevator(-1);
                }
            }
        }
    }

    @Override
    public void end() {
        elevators.moveLeftElevator(0);
        elevators.moveRightElevator(0);
        elevators.setBrakeMode();
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
