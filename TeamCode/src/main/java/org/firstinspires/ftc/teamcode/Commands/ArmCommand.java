package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmCommand implements Command {

    private double setPos;
    private double pivotPower;
    private PID pivotPID;
    public static double pos = 0; // Static variable
    private String name = "Arm";
    private ArmSubsystem arm;

    public ArmCommand(ArmSubsystem arm, double targetPos) {
        this.arm = arm;
        this.setPos = targetPos;
        this.pivotPID = new PID(0.15, 0, 0.01);
        this.pivotPID.setSetPoint(targetPos);
        this.pivotPID.setMaxOutput(0.5);
        this.pivotPID.setMinInput(-180);
        this.pivotPID.setMaxInput(180);

        System.out.println("Created ArmCommand with TargetPos: " + targetPos);
    }

    @Override
    public void start() {
        // Initialization logic if needed
    }

    @Override
    public void execute() {
        pos = arm.getCurrentPositionWithLimitSwitch(); // Update static pos
        pivotPower = pivotPID.updatePID(pos);
        arm.setPower(-pivotPower);
    }

    @Override
    public void end() {
        arm.setPower(0);
        System.out.println("Command ended.");
    }

    @Override
    public boolean isFinished() {
        double tolerance = 7;
        double currentPosition = arm.getCurrentPositionWithLimitSwitch();
        return Math.abs(currentPosition - setPos) <= tolerance;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return arm;
    }
}
