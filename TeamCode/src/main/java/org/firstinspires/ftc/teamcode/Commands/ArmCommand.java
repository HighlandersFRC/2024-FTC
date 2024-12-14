package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ArmCommand implements Command {

    public static double setPos;
    public static double pivotPower;



    public ArmCommand(double targetPos) {
        setPos = targetPos;
        piviotPID.setSetPoint(targetPos);
        piviotPID.setMaxOutput(0.5);
        piviotPID.setMinInput(180);
        piviotPID.setMaxInput(-180);
    }

    @Override
    public void start() {
System.out.println("Arm Executing");
    }

    @Override
    public void execute() {
        pivotPower = piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
        ArmSubsystem.setPower(-pivotPower);
    }

    @Override
    public void end() {
        ArmSubsystem.setPower(0);
        ArmSubsystem.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public boolean isFinished() {
        double tolerance = 7;
        double currentPosition = ArmSubsystem.getCurrentPositionWithLimitSwitch();

        return Math.abs(currentPosition - setPos) <= tolerance;
    }
}