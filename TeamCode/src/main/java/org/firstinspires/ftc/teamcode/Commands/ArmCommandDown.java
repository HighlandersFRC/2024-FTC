package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmCommandDown implements Command {
    public static final PID pivotPID = new PID(0.1, 0.004, 0.095);
    public static double setPos ;
    public static double pivotPower;



    public ArmCommandDown(double targetPos) {
        setPos = targetPos;
        pivotPID.setSetPoint(targetPos);
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinInput(180);
        pivotPID.setMaxInput(-180);
    }

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        pivotPower = pivotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
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
        return -Math.abs(ArmSubsystem.getCurrentPositionWithLimitSwitch() + tolerance) >= Math.abs(setPos);
    }
}