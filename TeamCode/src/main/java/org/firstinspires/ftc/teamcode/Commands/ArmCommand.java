package org.firstinspires.ftc.teamcode.Commands;
import static org.firstinspires.ftc.teamcode.Tools.Robot.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmCommand implements Command {

    private double setPos;
    private double pivotPower;
    private PID pivotPID;
    public static double Pos = 0;
    String name = "Arm";
    ArmSubsystem Arm;

    public ArmCommand(ArmSubsystem arm,double targetPos) {
        Arm=arm ;
        this.setPos = targetPos;
        this.pivotPID = new PID(0.015, 0, 0);
        this.pivotPID.setSetPoint(targetPos);
        this.pivotPID.setMaxOutput(0.5);
        this.pivotPID.setMinInput(-180);
        this.pivotPID.setMaxInput(180);

        System.out.println("Created ArmCommand with TargetPos: " + targetPos);
    }

    @Override
    public void start() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        Pos = ArmSubsystem.getCurrentPosition();
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
        double currentPosition = ArmSubsystem.getCurrentPositionWithLimitSwitch();
        return Math.abs(currentPosition - setPos) <= tolerance;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return Arm;
    }
}
