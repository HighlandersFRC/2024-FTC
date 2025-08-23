package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;

import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
@TeleOp
public class NewKitBot extends LinearOpMode {

NewArmSubsystem armSubsystem = new NewArmSubsystem("Arm", gamepad1);
NewElevatorSubsystem elevatorSubsystem = new NewElevatorSubsystem("Elevator", gamepad1);
NewIntakeSubsystem intakeSubsystem = new NewIntakeSubsystem("Intake", gamepad1);
NewWristSubsystem wristSubsystem = new NewWristSubsystem("Wrist", gamepad1);

Superstructure superstructure = new Superstructure("Structure");
    @Override
    public void runOpMode() throws InterruptedException {
      armSubsystem.init(hardwareMap);
      wristSubsystem.init(hardwareMap);
      intakeSubsystem.init(hardwareMap);
      elevatorSubsystem.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            armSubsystem.periodic();
            elevatorSubsystem.periodic();
            intakeSubsystem.periodic();
            wristSubsystem.periodic();
            superstructure.periodic();


            if (gamepad2.left_bumper) {
                elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_EXTEND);
            } else if (gamepad2.right_bumper) {
                elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_RETRACT);
            } else {
                elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.IDLE);
            }

            if (gamepad2.left_trigger > 0) {
                intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.INTAKE);
            } else if (gamepad2.right_trigger > 0) {
                intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.OUTTAKE);
            }

            if (gamepad2.dpad_up) {
                wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_UP);
            } else if (gamepad2.dpad_down) {
                wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_DOWN);
            }

            if (gamepad2.a) {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_UP);
            } else if (gamepad2.b) {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_DOWN);
            } else if (gamepad2.x) {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.SPECIMEN);
            } else if (gamepad2.y) {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.HIGH_BUCKET);
            } else {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.IDLE);
            }

            telemetry.addData("Current Pos Right", intakeSubsystem.RightIntake.getPosition());
            telemetry.addData("Current Pos Left", intakeSubsystem.LeftIntake.getPosition());
            telemetry.update();

        }
    }
}
