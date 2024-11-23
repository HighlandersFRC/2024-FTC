package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.PID;

public class Pivot extends Subsystem{
    public static PID pid = new PID(0.3, 0.0, 0.0);
    public static DcMotor pivotMotor;

    public static void initialize(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot");
    }

    public static void setPower(double power) {
        pivotMotor.setPower(power);
    }

    public static void stop() {
        setPower(0);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoder() {
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static int getEncoderPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public static void runUsingPID(double offsetPosition){
        pid.setSetPoint(offsetPosition);
    }

    public static void run(){
        pid.updatePID(getEncoderPosition());

        setPower(pid.getResult());
    }
}
