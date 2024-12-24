package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@TeleOp
public class kitbot extends LinearOpMode {
    public static double pos;
    public void runOpMode() throws InterruptedException {
        boolean armControlToggle = true;
        boolean togglePressed = false;
        waitForStart();
        ArmSubsystem.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        while (opModeIsActive()) {

            if (gamepad1.left_stick_button && !togglePressed) {
                armControlToggle = !armControlToggle;
                togglePressed = true;
            } else if (!gamepad1.left_stick_button) {
                togglePressed = false;
            }

            if (armControlToggle) {
                ArmSubsystem.ArmMovement(gamepad2);
                IntakeSubsystem.contolIntakeBlueAlliance(gamepad2);
                Wrist.controlWrist(gamepad2);
                if (!gamepad2.right_bumper || !gamepad2.left_bumper) {
                    if (gamepad2.y) {
                        pos = DegreesToEncoderTicks(120);
                    } else if (gamepad2.x) {
                        pos = DegreesToEncoderTicks(190);
                    } else if (gamepad2.dpad_down) {
                        pos = DegreesToEncoderTicks(215);
                    } else if (gamepad2.b) {
                        pos = DegreesToEncoderTicks(0);
                    }

                    piviotPID.setSetPoint(pos);
                    piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
                    piviotPID.setMaxOutput(0.7);
                    piviotPID.setMinOutput(-0.7);
                    ArmSubsystem.setPower(-piviotPID.getResult());
                }
            } else {
                ArmSubsystem.ArmMovement(gamepad1);
                IntakeSubsystem.contolIntakeBlueAlliance(gamepad1);
                Wrist.controlWrist(gamepad1);
                if (!gamepad1.right_bumper || !gamepad1.left_bumper) {
                    if (gamepad1.y) {
                        pos = DegreesToEncoderTicks(120);
                    } else if (gamepad1.x) {
                        pos = DegreesToEncoderTicks(190);
                    } else if (gamepad1.dpad_down) {
                        pos = DegreesToEncoderTicks(215);
                    } else if (gamepad1.b) {
                        pos = DegreesToEncoderTicks(0);
                    }

                    piviotPID.setSetPoint(pos);
                    piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
                    piviotPID.setMaxOutput(0.7);
                    piviotPID.setMinOutput(-0.7);
                    ArmSubsystem.setPower(-piviotPID.getResult());
                }
                gamepad2.rumble(1000);
            }


            Drive.FeildCentric(gamepad1);
            Mouse.update();

//            telemetry.addData("Mouse X", Mouse.getX());
//            telemetry.addData("Mouse Y", Mouse.getY());
//            telemetry.addData("Mouse θ", Mouse.getTheta());
//            telemetry.addData("Drive Left Front Pos", Drive.leftFrontPos());
//            telemetry.addData("Drive Right Front Pos", Drive.RightFrontPos());
//            telemetry.addData("Drive Left Back Pos", Drive.leftBackPos());
//            telemetry.addData("Drive Right Back Pos", Drive.RightBackPos());
//            telemetry.addData("Arm Current Position", getDegrees());

            telemetry.update();
        }
    }
}
