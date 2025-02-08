package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.PivotMove;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class PivotDefault implements Command {
    public static final PID pivotPID = new PID(0.09, 0.0, 0.0);
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
        if (Robot.PIVOT_STATE.equals("PID")){
        if (Elevators.getAvgEncoder() >= 1500){
            pivotPID.setSetPoint(Constants.ARM_HIGH);
        }
        if (Pivot.getAngle() > 110) {
            move = false;
            setPos = Robot.CURRENT_PIVOT;

            pivotPID.setSetPoint(Robot.CURRENT_PIVOT);
            pivotPower = pivotPID.updatePID(Pivot.getAngle());
            Pivot.setPower(pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Math.toRadians(Pivot.getAngle()) + Constants.ARM_BALANCE_OFFSET)));
        }
        Pivot.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pivot.pivotMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        if (Robot.PIVOT_STATE.equals("Power")){
            Pivot.setPower(Robot.PIVOT_RAW_POWER);
        }
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