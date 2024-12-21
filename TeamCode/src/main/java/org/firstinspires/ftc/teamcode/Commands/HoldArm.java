package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class HoldArm implements Command{
    public static boolean HOLD;
    @Override
    public void start() {
        HOLD = false;
    }

    @Override
    public void execute() {
HOLD = false;
        ArmSubsystem.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void end() {
HOLD = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
