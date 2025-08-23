package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;


@TeleOp
public class TestClass extends LinearOpMode {
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
ArmSubsystem arm = new ArmSubsystem("Arm", hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.cross) { // X Button - Blue
                gamepad1.setLedColor(0, 0, 255, 1000000000);
            } else if (gamepad1.circle) { // O Button - Red
                gamepad1.setLedColor(255, 0, 0, 1000000000);
            } else if (gamepad1.triangle) { // Triangle - Green
                gamepad1.setLedColor(0, 255, 0, 1000000000);
            } else if (gamepad1.square) { // Square - Purple
                gamepad1.setLedColor(128, 0, 128, 1000000000);
            }
        }


    }
}