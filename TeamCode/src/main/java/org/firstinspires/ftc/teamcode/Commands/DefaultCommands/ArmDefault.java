
package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;

import static org.firstinspires.ftc.teamcode.Subsystems.Pivot.pivotMotor;
import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmDefault implements Command {

    private double setPos;
    private double pivotPower;
    public double pos = 0;
    private String name = "Arm";
    private ArmSubsystem arm;

    public ArmDefault(ArmSubsystem arm) {
        this.arm = arm;
        pivotPID.setMaxOutput(1);
        pivotPID.setMinOutput(-1);
    }

    @Override
    public void start() {
        pos = arm.getCurrentPositionWithLimitSwitch();
    }

    @Override
    public void execute() {
        arm.setPosition(pos);
    }

    @Override
    public void end() {
        arm.setPosition(pos);
        System.out.println("Command ended.");
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