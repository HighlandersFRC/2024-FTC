package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;


@TeleOp
public class NewKitBot extends LinearOpMode {

NewArmSubsystem armSubsystem = new NewArmSubsystem("arm");
NewElevatorSubsystem elevatorSubsystem = new NewElevatorSubsystem("elevator");
NewIntakeSubsystem intakeSubsystem = new NewIntakeSubsystem("intake");
NewWristSubsystem wrist = new NewWristSubsystem("wrist");

    @Override
    public void runOpMode() throws InterruptedException {

      Drive drive = new Drive("Drive",hardwareMap);
      armSubsystem.init(hardwareMap);
      elevatorSubsystem.init(hardwareMap);
      intakeSubsystem.init(hardwareMap);
      wrist.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
           armSubsystem.periodic();
           elevatorSubsystem.periodic();
           intakeSubsystem.periodic();
           wrist.periodic();


           if (gamepad2.a) {
                armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_UP);
           } else if (gamepad2.b) {
               armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_DOWN);;
           } else if (gamepad2.y) {
               armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.SPECIMEN);
           } else if (gamepad2.x) {
               armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.HIGH_BUCKET);
           } else {
               armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.DEFAULT);
           }

           if (gamepad2.left_bumper) {
               elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_EXTEND);
            } else if (gamepad2.right_bumper) {
               elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_RETRACT);
           } else {
               elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.DEFAULT);
           }
           if (gamepad2.right_trigger > 0) {
               intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.INTAKE);
           } else if (gamepad2.left_trigger > 0){
               intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.OUTTAKE);
           }
           if (gamepad2.dpad_up) {
               wrist.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_UP);
           } else if (gamepad2.dpad_down) {
               wrist.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_DOWN);
           }

drive.FeildCentric(gamepad1);
        }
    }
}
