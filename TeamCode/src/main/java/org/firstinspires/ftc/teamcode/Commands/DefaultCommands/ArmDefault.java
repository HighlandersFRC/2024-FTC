
package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import static org.firstinspires.ftc.teamcode.Subsystems.Pivot.pivotMotor;
import static org.firstinspires.ftc.teamcode.Tools.Constants.GravityTerm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

import static org.firstinspires.ftc.teamcode.Commands.ArmCommand.pos;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

public class ArmDefault implements Command {

    private double pivotPower;

    private String name = "Arm";
    private ArmSubsystem arm;

    public ArmDefault(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void start() {
        piviotPID.setSetPoint(-1 * Math.abs(pos));
        System.out.println("ArmDefault started");
    }

    @Override
    public void execute() {
        pivotPower = piviotPID.updatePID(arm.getCurrentPositionWithLimitSwitch());
        double feed = GravityTerm(arm.getCurrentPositionWithLimitSwitch());
      arm.setPower(pivotMotor,-pivotPower * feed);
        System.out.println(pos + " ArmDefault executing");
    }

    @Override
    public void end() {
       arm.setPower(pivotMotor,0);
        System.out.println("ArmDefault ended");
    }

    @Override
    public boolean isFinished() {
        double tolerance = 7;
        double currentPosition = arm.getCurrentPositionWithLimitSwitch();
        return Math.abs(currentPosition - pos) <= tolerance;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return arm;
    }
}
