package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import static org.firstinspires.ftc.teamcode.Commands.ArmCommand.Pos;
import static org.firstinspires.ftc.teamcode.Tools.Constants.GravityTerm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;


public class ArmDefault implements Command {


    private double pivotPower;
    private PID pivotPID ;
    String name = "Arm";
    ArmSubsystem Arm;


    @Override
    public void start() {
        pivotPID.setPID(.10,0,0.01);
        pivotPID.setSetPoint(Pos);
    }

    @Override
    public void execute() {
        Pos = ArmSubsystem.getCurrentPosition();
        pivotPower = pivotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
        ArmSubsystem.setPower(-pivotPower*GravityTerm(Pos));
    }

    @Override
    public void end() {
        ArmSubsystem.setPower(0);
        ArmSubsystem.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return Arm;
    }
}
