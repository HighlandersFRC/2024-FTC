package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
@TeleOp
public class NewKitBot extends LinearOpMode {

NewArmSubsystem armSubsystem = new NewArmSubsystem("Arm", gamepad1);
NewIntakeSubsystem intakeSubsystem = new NewIntakeSubsystem("Intake", gamepad1);
NewElevatorSubsystem elevatorSubsystem = new NewElevatorSubsystem("Elevator", gamepad1);
NewWristSubsystem wristSubsystem = new NewWristSubsystem("Wrist", gamepad1);
Superstructure superstructure = new Superstructure("Structure");
    @Override
    public void runOpMode() throws InterruptedException {
      armSubsystem.init(hardwareMap);
      intakeSubsystem.init(hardwareMap);
      elevatorSubsystem.init(hardwareMap);
      wristSubsystem.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            armSubsystem.periodic();
            intakeSubsystem.periodic();
            elevatorSubsystem.periodic();
            wristSubsystem.periodic();
            superstructure.periodic();


            if(gamepad1.right_bumper) {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_UP);
            } else if (gamepad1.left_bumper) {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_DOWN);
            } else {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.IDLE);
            }

            if (gamepad1.dpad_up) {
                wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_UP);
            } else if (gamepad1.dpad_down) {
                wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_DOWN);
            }

            if (gamepad1.right_trigger > 0) {
                intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.INTAKE);
            } else if (gamepad1.left_trigger > 0) {
                intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.OUTTAKE);
            }


            if (gamepad1.a) {
                elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_EXTEND);
            } else if (gamepad1.b) {
                elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_RETRACT);
            } else {
                elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.IDLE);
            }
        }
    }
}
