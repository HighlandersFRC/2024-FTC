package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;


@TeleOp
public class kitbot extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
    waitForStart();
    ArmSubsystem.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        Mouse.init(hardwareMap);

        while (opModeIsActive()) {
           ArmSubsystem.controlPivotWithOperator(gamepad2, piviotPID);
          IntakeSubsystem.contolIntake(gamepad2);
            Wrist.controlWrist(gamepad2);
            Drive.FeildCentric(gamepad1);
            ArmSubsystem.gamepad1Climb(gamepad1);
            Mouse.update();

            telemetry.addData("Mouse X", Mouse.getX());
            telemetry.addData("Mouse Y", Mouse.getY());
            telemetry.addData("Mouse θ", Mouse.getTheta());
            telemetry.addData("Drive train left front", Drive.leftFrontPos());
            telemetry.addData("Drive train right front", Drive.RightFrontPos());
            telemetry.addData("Drive train left back", Drive.leftBackPos());
            telemetry.addData("Drive train right back", Drive.RightBackPos());
            telemetry.addData("Pivot Arm Current Pos", ArmSubsystem.getCurrentPosition());
            telemetry.addData("limit",ArmSubsystem.limitSwitch());
            telemetry.addData("logic", gamepad1.right_trigger!=0);
            telemetry.addData("right trigger", -gamepad1.right_trigger);
            telemetry.addData("power", IntakeSubsystem.getPower());
            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}