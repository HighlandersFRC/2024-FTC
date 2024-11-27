package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.PID;

public class Pivot extends Subsystem {


    private static final PID pid = new PID(0.3, 0.0, 0.0);

    private static DcMotor pivotMotor;


    public static void initialize(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public static void setPower(double power) {
        pivotMotor.setPower(power);
    }

    public static void stop() {
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setPower(0);
    }


    public static void resetEncoder() {
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public static int getEncoderPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public static void runUsingPID(double targetPosition) {
        pid.setSetPoint(targetPosition);
    }


    public static void run() {
        double correction = pid.updatePID(getEncoderPosition());
        setPower(correction);
    }
}
