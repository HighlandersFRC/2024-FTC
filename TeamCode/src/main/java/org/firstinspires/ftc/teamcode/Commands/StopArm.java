package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class StopArm implements Command{
    private ArmSubsystem arm;
    public StopArm(ArmSubsystem arm){
        this.arm = arm;
    }
    @Override
    public void start() {
        System.out.println("Stop arm started");
    }

    @Override
    public void execute() {
        arm.setPosition(arm.getCurrentPositionWithLimitSwitch());
arm.setPower(0);
    }

    @Override
    public void end() {
        arm.setPosition(arm.getCurrentPositionWithLimitSwitch());
        arm.setPower(0);
    }

    @Override
    public boolean isFinished() {
            return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return arm;
    }
}
