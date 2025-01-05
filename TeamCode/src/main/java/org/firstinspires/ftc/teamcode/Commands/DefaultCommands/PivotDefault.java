package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class PivotDefault implements Command {
    public static final PID pivotPID = new PID(0.09, 0.0, 0.0425);
    public static double setPos;
    public static double pivotPower;
    String name = "Pivot";
    Pivot pivotSubsystem = Robot.pivot;
    boolean move;

    @Override
    public void start() {

    }

    @Override
    public void execute() {

        if (Pivot.getAngle() > 90) {
            move = false;
            setPos = Robot.CURRENT_PIVOT;

            pivotPID.setSetPoint(Robot.CURRENT_PIVOT);
            pivotPower = pivotPID.updatePID(Pivot.getAngle());
            Pivot.setPower(pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Math.toRadians(Pivot.getAngle()) + Constants.ARM_BALANCE_OFFSET)));
        } else {
            move = true;
        }
        Pivot.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return move;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return pivotSubsystem;
    }
}