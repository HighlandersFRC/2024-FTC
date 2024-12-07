

package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevators {
    private static DcMotor leftElevator;
    private static DcMotor rightElevator;


    public static void initialize(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void moveUp() {
        leftElevator.setPower(1);
        rightElevator.setPower(1);
    }


    public static void moveDown() {
        leftElevator.setPower(-0.4);
        rightElevator.setPower(-0.4);
    }


    public static void stop() {
        leftElevator.setPower(0);
        rightElevator.setPower(0);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
