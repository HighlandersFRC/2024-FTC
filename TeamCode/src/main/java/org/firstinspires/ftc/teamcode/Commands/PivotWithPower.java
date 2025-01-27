package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class PivotWithPower implements Command {
    public static final PID pivotPID = new PID(0.002, 0.0, 0.0);
    public static double setPos;
    public static double pivotPower;
    String name = "Pivot";
    Pivot pivotSubsystem;

    public PivotWithPower(Pivot pivot, double power) {
        pivotSubsystem = pivot;
        setPos = power;
    }

    @Override
    public void start() {
        Robot.CURRENT_PIVOT = setPos;
    }

    @Override
    public void execute() {
        Pivot.setPower(pivotPower);
  ;
    }

    @Override
    public void end() {
        Pivot.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Pivot.getAngle() - setPos) <= (1);
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return pivotSubsystem;
    }
}