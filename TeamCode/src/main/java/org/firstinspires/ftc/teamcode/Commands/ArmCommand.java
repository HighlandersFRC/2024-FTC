
package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Subsystems.Pivot.pivotMotor;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.setPowerToPercentage;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmCommand implements Command {

    private double setPos;
    private double pivotPower;
    public static double pos = 0; // Static variable
    public double posToo = 0;
    private String name = "Arm";
    private ArmSubsystem arm;

    public ArmCommand(ArmSubsystem arm, double targetPos) {
        this.arm = arm;
        this.setPos = targetPos;
        piviotPID.setSetPoint(targetPos);
        piviotPID.setMaxOutput(setPowerToPercentage(80));
        piviotPID.setMinOutput(setPowerToPercentage(-80));

        System.out.println("Created ArmCommand with TargetPos: " + targetPos);

    }

    @Override
    public void start() {
        // Initialization logic if needed
    }

    @Override
    public void execute() {
//        pos = arm.getCurrentPositionWithLimitSwitch(); // Update static pos
//        posToo =arm.getCurrentPositionWithLimitSwitch(); // Update  posToo
        piviotPID.updatePID(arm.getCurrentPositionWithLimitSwitch());
        arm.setPower( -piviotPID.getResult());
        System.out.println("Pivot PID: " + piviotPID.getResult());
        System.out.println("Set Pos" + setPos);
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