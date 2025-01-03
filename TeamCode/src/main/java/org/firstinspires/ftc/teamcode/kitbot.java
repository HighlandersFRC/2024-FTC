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
public class kitbot extends LinearOpMode {
    public boolean armControlToggle = true;
    public boolean togglePressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem("Elevator", hardwareMap);
        ArmSubsystem armSubsystem = new ArmSubsystem("Arm", hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem("Intake", hardwareMap);
        Wrist wristSubsystem = new Wrist("Wrist", hardwareMap);
        Drive driveSubsystem = new Drive("Drive", hardwareMap);


        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.touchpad && !togglePressed) {
                armControlToggle = !armControlToggle;
                togglePressed = true;
            } else if (!gamepad1.touchpad) {
                togglePressed = false;
            }





            if (armControlToggle) {
                armSubsystem.ArmMovement(gamepad2);
                elevatorSubsystem.contolElevator(gamepad2);
                intakeSubsystem.controlIntake(gamepad2);
                wristSubsystem.controlWrist(gamepad2);
            } else {
                armSubsystem.ArmMovement(gamepad1);
                elevatorSubsystem.contolElevator(gamepad1);
                intakeSubsystem.controlIntake(gamepad1);
                wristSubsystem.controlWrist(gamepad1);
                gamepad2.rumble(1000);
            }

            Mouse.update();
            driveSubsystem.FeildCentric(gamepad1);
            telemetry.addData("Gamepad Toggle State", armControlToggle ? "Gamepad2" : "Gamepad1");
            telemetry.addData("Arm Degrees", getDegrees(armSubsystem.getCurrentPositionWithLimitSwitch()));
            telemetry.addData("Drive Degrees", getDegrees(driveSubsystem.leftBackPos()));
            telemetry.addData("Wrist Pos", wristSubsystem.getPosition());
            telemetry.update();
        }
    }
}