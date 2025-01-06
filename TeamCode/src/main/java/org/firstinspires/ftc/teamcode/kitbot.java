package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.absoluteArmZero;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.setPowerToPercentage;

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
    @Override
    public void runOpMode() throws InterruptedException {
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
                rumble = 0;
            } else {
                armSubsystem.manual(gamepad1);
                intakeSubsystem.controlIntake(gamepad1);
                elevatorSubsystem.manual(gamepad1);
                rumble = 1000;
            }
            gamepad2.rumble(rumble);
            gamepad1.rumble(rumble);
            Mouse.update();
            driveSubsystem.FeildCentric(gamepad1);
            double wristPosition = 0.65;
            if (gamepad1.b) {
               wristPosition = 0.55;
            }
            if (elevatorSubsystem.getCurrentPosition() <= -4000) {
                elevatorSubsystem.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevatorSubsystem.setPower(0);
            } else {
                if (gamepad1.right_bumper) {
                    elevatorSubsystem.setPower(1);
                } else if (gamepad1.left_bumper) {
                    elevatorSubsystem.setPower(-1);
                } else {
                    elevatorSubsystem.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    elevatorSubsystem.setPower(0);
                }
            }
            wristSubsystem.setPosition(wristPosition);
            //Manual Arm Movement (if arm is manual)
            elevatorSubsystem.setPower(armSubsystem.elePos);
            //PID movement
//            elevatorSubsystem.setPosition(armSubsystem.elePos);
            telemetry.addData("Right Intake Current Pos", intakeSubsystem.getPositionRight());
            telemetry.addData("Left Intake Current Pos", intakeSubsystem.getPositionLeft());
            telemetry.addData("Gamepad Toggle State", armControlToggle ? "Gamepad2" : "Gamepad1");
            telemetry.addData("Arm Degrees", getDegrees(armSubsystem.getCurrentPositionWithLimitSwitch()));
            telemetry.addData("Drive Degrees", getDegrees(driveSubsystem.leftBackPos()));
            telemetry.addData("Wrist Pos", wristSubsystem.getPosition());
            telemetry.update();
        }
    }
}