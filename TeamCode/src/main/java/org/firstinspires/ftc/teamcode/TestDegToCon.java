package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.encodersToDeg;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;



@TeleOp
public class TestDegToCon extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ArmSubsystem.initialize(hardwareMap);

        while (opModeIsActive()) {

            piviotPID.setSetPoint(encodersToDeg(90));
            piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
            piviotPID.setMaxOutput(1);
            piviotPID.setMinOutput(-1);
            ArmSubsystem.setPower(-piviotPID.getResult());

            telemetry.addData("ArmCurrentPos", ArmSubsystem.getCurrentPositionWithLimitSwitch());
            telemetry.addData("ArmCurrentPos Deg", getDegrees());
            telemetry.update();

        }
    }
}