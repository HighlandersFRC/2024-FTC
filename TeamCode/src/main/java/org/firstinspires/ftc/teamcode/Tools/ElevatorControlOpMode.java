/*
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

@TeleOp
public class ElevatorControlOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
       ElevatorSubsystem elevator = new ElevatorSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                elevator.moveUp();
            } else if (gamepad1.left_bumper) {
                elevator.moveDown();
            } else if (gamepad1.right_trigger > 0.1) {
                elevator.stop();
            } else {
                elevator.stop();
            }
        }
    }
}
*/

package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

@TeleOp
public class ElevatorControlOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSubsystem elevator = new ElevatorSubsystem(hardwareMap);
        boolean isMoving = false;

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                ElevatorSubsystem.run();
            }
          if(gamepad1.left_bumper){
              ElevatorSubsystem.down();
          }
          if(gamepad1.right_bumper){
              ElevatorSubsystem.up();
          }

telemetry.addData("pivot", ElevatorSubsystem.getCurrentPivotPosition());
            telemetry.addData("Current Position Left", ElevatorSubsystem.getCurrentLeftPosition());
            telemetry.addData("Current Position Right", ElevatorSubsystem.getCurrentRightPosition());

            telemetry.update();
        }
    }
}
