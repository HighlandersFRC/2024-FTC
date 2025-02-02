package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class ManualPivot implements Command {
    private Pivot pivotSubsystem;
    private double power;

    private long timeStart;

    public ManualPivot(Pivot pivot, double power) {
        this.pivotSubsystem = pivot;
        this.power = power;
    }

    @Override
    public void start() {
timeStart=System.currentTimeMillis();
    }

    @Override
    public void execute() {
        pivotSubsystem.setPower(power);
    }

    @Override
    public void end() {
        pivotSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis()-timeStart) >= 1000;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return pivotSubsystem;
    }
}
