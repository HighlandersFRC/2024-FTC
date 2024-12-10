package org.firstinspires.ftc.teamcode.Commands;


import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ArmCommmandDown implements Command {
    double armPosition;
    double tolerance = 7;
    @Override
    public void start() {
        System.out.println("Down Started");
    }

    @Override
    public void execute() {
System.out.println("Down Executing");
        armPosition = 0;
        piviotPID.setSetPoint(armPosition);
        piviotPID.setMaxOutput(1);
        piviotPID.setMinOutput(-1);
        piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
        ArmSubsystem.pivotMotor.setPower(-piviotPID.getResult());
    }

    @Override
    public void end() {
    ArmSubsystem.pivotMotor.setPower(0);
    ArmSubsystem.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public boolean isFinished() {
        // Must Get the == to >= to work
        if (Math.abs(ArmSubsystem.getCurrentPositionWithLimitSwitch() + tolerance) == Math.abs(armPosition)) {
            return true;
        }
        return false;
    }
}
