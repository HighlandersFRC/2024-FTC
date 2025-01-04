package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.absoluteArmZero;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;


@TeleOp
public class TestClass extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem("Elevator", hardwareMap);
        Drive driveSubsystem = new Drive("Drive", hardwareMap);

        waitForStart();


        while (opModeIsActive()) {
        driveSubsystem.FeildCentric(gamepad1);
                elevatorSubsystem.contolElevatorSetPoint(gamepad1);

            telemetry.update();
        }
    }
}