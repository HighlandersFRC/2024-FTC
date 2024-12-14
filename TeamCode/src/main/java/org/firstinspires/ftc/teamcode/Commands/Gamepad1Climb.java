package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class Gamepad1Climb implements Command{
    public double Arm_power;
    public boolean STOP;

    @Override
    public void start() {
        STOP = false;
    }
public Gamepad1Climb(double power) {
        power = Arm_power;
    STOP = false;
}

    @Override
    public void execute() {
        ArmSubsystem.setPower(Arm_power);
        STOP = false;
    }

    @Override
    public void end() {
        STOP = true;
    }

    @Override
    public boolean isFinished() {
        return STOP;
    }
}
