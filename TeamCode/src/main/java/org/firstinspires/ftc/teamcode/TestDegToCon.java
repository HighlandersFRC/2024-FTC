package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
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
        ArmSubsystem armSubsystem = new ArmSubsystem("arm",hardwareMap);

        while (opModeIsActive()) {


            if (gamepad1.y) {
                pos = DegreesToEncoderTicks(120);
            } else if (gamepad1.x) {
                pos = DegreesToEncoderTicks(190);
            } else if (gamepad1.dpad_down) {
                pos = DegreesToEncoderTicks(215);
            } else if(gamepad1.b) {
                pos = DegreesToEncoderTicks(0);
            }
            piviotPID.setSetPoint(pos);
            piviotPID.updatePID(armSubsystem.getCurrentPositionWithLimitSwitch());
            piviotPID.setMaxOutput(1);
            piviotPID.setMinOutput(-1);
            armSubsystem.setPower(-piviotPID.getResult());

            telemetry.addData("ArmCurrentPos", armSubsystem.getCurrentPositionWithLimitSwitch());
            telemetry.addData("ArmCurrentPos Deg", getDegrees() + "°");
            telemetry.update();

        }
    }
}