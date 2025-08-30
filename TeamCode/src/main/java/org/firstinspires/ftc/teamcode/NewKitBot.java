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

Superstructure superstructure = new Superstructure("superstructure");

    @Override
    public void runOpMode() throws InterruptedException {

      Drive drive = new Drive("Drive",hardwareMap);
      superstructure.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
           superstructure.periodic();

           if (gamepad2.a) {
                superstructure.setWantedState(Superstructure.SUPER_STATE.ARM_UP);
           } else if (gamepad2.b) {
                superstructure.setWantedState(Superstructure.SUPER_STATE.ARM_DOWN);
           } else if (gamepad2.y) {
               superstructure.setWantedState(Superstructure.SUPER_STATE.ARM_HIGH_BUCKET);
           } else if (gamepad2.x) {
               superstructure.setWantedState(Superstructure.SUPER_STATE.ARM_SPECIMEN);
           } else if (gamepad2.left_bumper) {
               superstructure.setWantedState(Superstructure.SUPER_STATE.ELEVATOR_EXTEND);
            } else if (gamepad2.right_bumper) {
               superstructure.setWantedState(Superstructure.SUPER_STATE.ELEVATOR_RETRACT);
           } else if (gamepad2.right_trigger > 0) {
               superstructure.setWantedState(Superstructure.SUPER_STATE.INTAKE);
           } else if (gamepad2.left_trigger > 0){
               superstructure.setWantedState(Superstructure.SUPER_STATE.OUTTAKE);
           } else if (gamepad2.dpad_up) {
               superstructure.setWantedState(Superstructure.SUPER_STATE.WRIST_UP);
           } else if (gamepad2.dpad_down) {
               superstructure.setWantedState(Superstructure.SUPER_STATE.WRIST_DOWN);
           } else {
               superstructure.setWantedState(Superstructure.SUPER_STATE.DEFAULT);
           }




drive.FeildCentric(gamepad1);
        }
    }
}
