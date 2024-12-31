package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;

@TeleOp
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");
        Mouse.init(hardwareMap);

        waitForStart();
        PID yawPID = new PID(2.5, 0, 1);

        double strafe = yawPID.updatePID(Math.toRadians(Mouse.getTheta()));

        double turn90 = Math.toRadians(DegreesToEncoderTicks(90));


        yawPID.setSetPoint(turn90);


        yawPID.setMinOutput(-1);
        yawPID.setMaxOutput(1);


        while (opModeIsActive()) {
            double frontLeftPower = (-strafe);
            double backLeftPower = (strafe);
            double frontRightPower = (strafe);
            double backRightPower = (-strafe);

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);
            telemetry.addData("Yaw Error", yawPID.getResult());
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Target Position", turn90);
            telemetry.addData("Front Left Position", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Position", frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Left Position", backLeftMotor.getCurrentPosition());
            telemetry.addData("Back Right Position", backRightMotor.getCurrentPosition());
            telemetry.addData("Theta Raw Data", "Mouse.getTheta(): " + Mouse.getTheta());
            telemetry.update();
        }
    }
}
