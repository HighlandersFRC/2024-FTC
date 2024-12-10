package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class PivotMove implements Command {
    public static final PID pivotPID = new PID(0.1, 0.004, 0.095);
    public static double setPos;
    public static double pivotPower;
    String name = "Pivot";


    public PivotMove(double targetPos) {
        setPos = targetPos;
        pivotPID.setSetPoint(targetPos);
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinInput(180);
        pivotPID.setMaxInput(-180);
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
        pivotPower = pivotPID.updatePID(Pivot.getAngle());
        Pivot.setPower(pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Math.toRadians(Pivot.getAngle()) + Constants.ARM_BALANCE_OFFSET)));
    }

    @Override
    public void end() {
        Pivot.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Pivot.getAngle() - setPos) <= (1);
    }
}