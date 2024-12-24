package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.encodersToDeg;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;


@TeleOp
public class TestDegToCon extends LinearOpMode {
    public static double pos;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ArmSubsystem.initialize(hardwareMap);

        while (opModeIsActive()) {


            if (gamepad1.y) {
                pos = encodersToDeg(120);
            } else if (gamepad1.x) {
                pos = encodersToDeg(190);
            } else if (gamepad1.dpad_down) {
                pos = encodersToDeg(215);
            } else if(gamepad1.b) {
                pos = encodersToDeg(0);
            }
            piviotPID.setSetPoint(pos);
            piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
            piviotPID.setMaxOutput(1);
            piviotPID.setMinOutput(-1);
            ArmSubsystem.setPower(-piviotPID.getResult());

            telemetry.addData("ArmCurrentPos", ArmSubsystem.getCurrentPositionWithLimitSwitch());
            telemetry.addData("ArmCurrentPos Deg", getDegrees(ArmSubsystem.getCurrentPositionWithLimitSwitch()) + "°");
            telemetry.update();

        }
    }
}