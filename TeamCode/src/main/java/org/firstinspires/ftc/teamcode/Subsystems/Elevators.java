package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.Constants;

public class Elevators extends Subsystem {
    private static DcMotor leftElevator;
    private static DcMotor rightElevator;

    private static final int STALL_THRESHOLD = 10;
    private static final long MONITOR_INTERVAL_MS = 100;

    public static void initialize(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");
        resetEncoders();
        setBrakeMode();
    }

    public static void moveUp() {
       if(Pivot.getAngle() <= 0){
           if(getAverage() <= Constants.ELEVATOR_MAX){
               setPower(0.8);
           }else {
               stop();
           }
       }else{
           setPower(0.8);
       }
    }

    public static void moveDown() {
        setPower(-0.65);
    }

    public static void stop() {
        setPower(0.0);
        setBrakeMode();
    }

    public static void resetEncoders() {
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static double getLeftEncoder() {
        return leftElevator.getCurrentPosition();
    }

    public static double getRightEncoder() {
        return rightElevator.getCurrentPosition();
    }
public static double getAverage(){
        return (leftElevator.getCurrentPosition() + rightElevator.getCurrentPosition())/2;
}
    private static void setPower(double power) {
        leftElevator.setPower(power);
        rightElevator.setPower(power);
    }

    public static void setBrakeMode() {
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
