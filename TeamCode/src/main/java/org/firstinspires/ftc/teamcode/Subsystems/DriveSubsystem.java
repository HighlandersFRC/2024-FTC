package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends Subsystem {

    public static DcMotor LeftMotor;
    public static DcMotor RightMotor;

    public DriveSubsystem(String name) {
        super(name);
    }
    public static void start(HardwareMap hardwareMap){
        LeftMotor = hardwareMap.dcMotor.get("LeftMotor");
        RightMotor = hardwareMap.dcMotor.get("RightMotor");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();
        brakeMotors();
    }
    @Override
    public void initialize(HardwareMap hardwareMap) {
        LeftMotor = hardwareMap.dcMotor.get("LeftMotor");
        RightMotor = hardwareMap.dcMotor.get("RightMotor");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();
        brakeMotors(); // Ensure motors brake when power is zero
    }

    public static void Drive(double LeftMotorPower, double RightMotorPower) {
        LeftMotor.setPower(LeftMotorPower);
        RightMotor.setPower(RightMotorPower);
    }

    public static void resetEncoders() {
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void brakeMotors() {
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void floatMotors() {
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static DcMotor getRight() {
        return RightMotor;
    }

    public static DcMotor getLeft() {
        return LeftMotor;
    }

    public static double getRightEncoder() {
        return RightMotor.getCurrentPosition();
    }

    public static double getLeftEncoder() {
        return LeftMotor.getCurrentPosition();
    }
}
