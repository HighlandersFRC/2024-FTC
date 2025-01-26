
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
    public  double pos = 0; // Static variable
    public double posToo = 0;
    private String name = "Arm";
    private ArmSubsystem arm;

    public ArmCommand(ArmSubsystem arm, double targetPos) {
        this.arm = arm;
        this.setPos = targetPos;
    }

    @Override
    public void start() {
        System.out.println(setPos);
        System.out.println("Created ArmCommand with TargetPos: " + setPos + " Current Position: " + arm.getCurrentPositionWithLimitSwitch());
    }

    @Override
    public void execute() {
//        pos = arm.getCurrentPositionWithLimitSwitch(); // Update static pos
//        posToo =arm.getCurrentPositionWithLimitSwitch(); // Update  posToo
System.out.println("Executing");
        arm.setPosition(setPos);
    }

    @Override
    public void end() {
//        piviotPID.setSetPoint(arm.getCurrentPositionWithLimitSwitch());
//        piviotPID.updatePID(arm.getCurrentPositionWithLimitSwitch());
//        piviotPID.setMaxOutput(1);
//        piviotPID.setMinOutput(-1);
//        arm.setPower(piviotPID.getResult());

        arm.setZeroPowerBehavior();
        System.out.println("Command ended.");
        System.out.println("serdtgf");
    }

    @Override
    public boolean isFinished() {
        double tolerance = 10;
        double currentPosition = arm.getCurrentPositionWithLimitSwitch();
        return Math.abs(currentPosition - setPos) <= tolerance;


    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return arm;
    }
}