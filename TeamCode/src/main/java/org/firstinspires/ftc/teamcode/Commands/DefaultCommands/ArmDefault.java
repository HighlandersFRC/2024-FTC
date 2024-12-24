package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.GravityTerm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;
import static org.firstinspires.ftc.teamcode.Commands.ArmCommand.Pos;


public class ArmDefault implements Command {

    private double pivotPower;
    private PID pivotPID = new PID(.001,0,0) ;
    String name = "Arm";
    ArmSubsystem Arm;



    @Override
    public void start() {
        pivotPID.setSetPoint(-1*Math.abs(Pos));
        System.out.println("default");
    }

    @Override
    public void execute() {
        pivotPower = pivotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
        double feed = GravityTerm(ArmSubsystem.getCurrentPositionWithLimitSwitch());
        ArmSubsystem.setPower(-pivotPower*feed);
        System.out.println(Pos+"default");
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
        return Math.abs(currentPosition - Pos) <= tolerance;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return Arm;
    }
}
