package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class Pivot extends Subsystem{
    private static final PID pid = new PID(0.009, 0.0, 0.0092);
    public static DcMotor pivotMotor;
    public static DigitalChannel limitSwitch;

    public static void initialize(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot");
/*
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit_switch");
*/

        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoder();
    }

/*    public static void checkForZero(){
    if (limitSwitch.getState()){
        resetEncoder();
    }
    }*/

    public static void setPower(double power) {
        pivotMotor.setPower(power);
    }

    public static void stop() {
        setPower(0);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void resetEncoder() {
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
        double pivotPower = pid.updatePID(Pivot.getAngle());

        setPower(pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Pivot.getAngle() + Constants.ARM_BALANCE_OFFSET)));
    }
    public static double ticksToDegrees(double ticks) {
        double degrees = (ticks / Constants.PIVOT_TICKS_PER_ROTATION) * 360.0;
        return degrees + Constants.PIVOT_STARTING_ANGLE;
    }
    public static double getAngle(){
     return ((getEncoderPosition()) / (678 / 90.8)) - 14.8;
    }
}
