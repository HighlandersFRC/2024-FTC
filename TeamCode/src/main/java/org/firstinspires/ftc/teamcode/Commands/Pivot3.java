package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class Pivot3 implements Command {
    public static final PID pivotPID = new PID(0.008, 0.0, 0.0);
    public static double setPos;
    public static double pivotPower;
    String name = "Pivot";
    Pivot pivotSubsystem;

    public Pivot3(Pivot pivot, double targetPos) {
        pivotSubsystem = pivot;
        setPos = targetPos;
        pivotPID.setSetPoint(targetPos);
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinInput(180);
        pivotPID.setMaxInput(-180);
    }

    @Override
    public void start() {
        Robot.CURRENT_PIVOT = setPos;
    }

    @Override
    public void execute() {
        pivotPower = pivotPID.updatePID(Pivot.getAngle());
        pivotPower += (Constants.PIVOT_FEED_FORWARD * Math.cos(Math.toRadians(Pivot.getAngle()) + Constants.ARM_BALANCE_OFFSET));
        Pivot.setPower(pivotPower);
        RobotLog.d("Pivot power: " +  pivotPower);
    }

    @Override
    public void end() {
        Pivot.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Pivot.getAngle() - setPos) <= (3);
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}