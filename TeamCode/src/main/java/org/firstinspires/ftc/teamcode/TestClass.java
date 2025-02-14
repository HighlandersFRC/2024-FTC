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
       ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem("Elevator", hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
           elevatorSubsystem.contolElevatorSetPoint(gamepad1);
        }


    }
}