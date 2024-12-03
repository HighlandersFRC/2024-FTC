package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class PivotMove implements Command {
    private final PID pivotPID = new PID(0.0065, 0.004, 0.0092);
    public static double setPos;

    public PivotMove(double targetPos) {
        setPos = targetPos;
        pivotPID.setSetPoint(targetPos);
    }

    @Override
    public void start() {
        // Initialization logic if needed before command execution starts
        Pivot.resetEncoder();
    }

    @Override
    public void execute() {
        double power = pivotPID.updatePID(Pivot.getEncoderPosition());
        Pivot.setPower(power);
    }

    @Override
    public void end() {
        // Logic to stop the Pivot when the command ends
        Pivot.setPower(0);
    }

    @Override
    public boolean isFinished() {
        // Check if the Pivot has reached the target position within a tolerance
        return Math.abs(Pivot.getEncoderPosition() - setPos) <= 10;
    }
}