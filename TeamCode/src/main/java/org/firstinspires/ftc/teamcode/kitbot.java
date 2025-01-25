package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;


@TeleOp
public class kitbot extends LinearOpMode {
    public int rumble;
    public boolean armControlToggle = true;
    public boolean togglePressed = false;
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem("Elevator", hardwareMap);
        ArmSubsystem armSubsystem = new ArmSubsystem("Arm", hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem("Intake", hardwareMap);
        Wrist wristSubsystem = new Wrist("Wrist", hardwareMap);
        Drive driveSubsystem = new Drive("Drive", hardwareMap);


        waitForStart();
Mouse.configureOtos();

        while (opModeIsActive()) {

            if (gamepad1.touchpad && !togglePressed) {
                armControlToggle = !armControlToggle;
                togglePressed = true;
            } else if (!gamepad1.touchpad) {
                togglePressed = false;
            }



            if (armControlToggle) {
                armSubsystem.manual(gamepad2);
                intakeSubsystem.controlIntake(gamepad2);
                elevatorSubsystem.manual(gamepad2);
                wristSubsystem.contolWrist(gamepad2);
                rumble = 0;
            } else {
                armSubsystem.manual(gamepad1);
                intakeSubsystem.controlIntake(gamepad1);
                elevatorSubsystem.manual(gamepad1);
                wristSubsystem.contolWrist(gamepad1);
                rumble = 1000;
            }

                    gamepad2.rumble(rumble);
            Mouse.update();

            driveSubsystem.FeildCentric(gamepad1);


//            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0 ) {
//                intakeSubsystem.setPosition(armSubsystem.intakePosition);
//            }
//wristSubsystem.setPosition(armSubsystem.wristPosition);
            //Manual Arm Movement (if arm is manual)
//            elevatorSubsystem.setPower(armSubsystem.elePos);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Right Intake Current Pos", intakeSubsystem.getPositionRight());
            packet.put("Left Intake Current Pos", intakeSubsystem.getPositionLeft());
            packet.put("Gamepad Toggle State", armControlToggle ? "Gamepad2" : "Gamepad1");
            packet.put("Arm Degrees", getDegrees(armSubsystem.getCurrentPositionWithLimitSwitch()));
            packet.put("Drive Degrees", getDegrees(driveSubsystem.leftBackPos()));
            packet.put("Wrist Pos", wristSubsystem.getPosition());
            packet.put("Mouse Sensor X", -Mouse.getY());
            packet.put("Mouse Sensor Y", -Mouse.getX());
            packet.put("Mouse Sensor theta", -Mouse.getTheta());
            dashboard.sendTelemetryPacket(packet);
            //PID movement
//            elevatorSubsystem.setPosition(armSubsystem.elePos);
            telemetry.addData("Right Intake Current Pos", intakeSubsystem.getPositionRight());
            telemetry.addData("Left Intake Current Pos", intakeSubsystem.getPositionLeft());
            telemetry.addData("Gamepad Toggle State", armControlToggle ? "Gamepad2" : "Gamepad1");
            telemetry.addData("Arm Degrees", getDegrees(armSubsystem.getCurrentPositionWithLimitSwitch()));
            telemetry.addData("elevator",elevatorSubsystem.getCurrentPosition());
            telemetry.addData("Drive Degrees", getDegrees(driveSubsystem.leftBackPos()));
            telemetry.addData("Wrist Pos", wristSubsystem.getPosition());
            telemetry.addData("Mouse Sensor X", -Mouse.getY());
            telemetry.addData("Mouse Sensor Y", -Mouse.getX());
            telemetry.addData("Mouse Sensor theta", -Mouse.getTheta());
            telemetry.update();
        }
    }
}