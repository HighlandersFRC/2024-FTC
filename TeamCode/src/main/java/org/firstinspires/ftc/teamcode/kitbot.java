package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

@TeleOp
public class kitbot extends LinearOpMode {
    PID piviotPID = new PID(0.15, 0, 1);
    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        //DigitalChannel LimitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch");

        double armPosition = 0;
        piviotPID.updatePID(pivotMotor.getCurrentPosition());
        piviotPID.setMaxOutput(1);
        piviotPID.setMinOutput(-1);
        piviotPID.setSetPoint(armPosition);
        pivotMotor.setPower(piviotPID.getResult());

//        if (LimitSwitch.getState() == true) {
//            armPosition = 1736;
//        } else if (LimitSwitch.getState() == false) {
//            armPosition = 0;
//        }


        waitForStart();
        Drive.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                Wrist.setPower(1);
            } else if (gamepad1.right_bumper) {
                Wrist.setPower(1);
            } else {
                Wrist.setPower(0);
            }

            if (gamepad1.left_trigger != 0) {
                IntakeSubsystem.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger != 0) {
               IntakeSubsystem.setPower(-gamepad1.right_trigger);
            } else {
               IntakeSubsystem.setPower(0);
            }

            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(pivot), 1);


            double frontLeftPower = ((forward - strafe  - pivot) / max);
            double BackLeftPower = ((forward + strafe  - pivot) / max);
            double FrontRightPower = ((forward - strafe  + pivot) / max);
            double BackRightPower =(( forward + strafe + pivot) / max);

            Drive.drive(frontLeftPower,FrontRightPower,BackLeftPower,BackRightPower);

            if (frontLeftPower==0){
                Drive.stop();
            }

            piviotPID.setSetPoint(armPosition);
            piviotPID.setMaxOutput(1);
            piviotPID.setMinOutput(-1);
            piviotPID.updatePID(pivotMotor.getCurrentPosition());
            pivotMotor.setPower(-piviotPID.getResult());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}







