/*

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem {
    private final DcMotor leftElevator;
    private final DcMotor rightElevator;
    private final DcMotor pivot;

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");
        pivot         = hardwareMap.get(DcMotor.class, "pivot");
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
public void PivotUp(){
        pivot.setPower(0.7);
}

public void PivotDown(){
        pivot.setPower(-0.7);
}
public void PivotStop(){
        pivot.setPower(0);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}
    public void moveUp() {
        leftElevator.setPower(0.4);
        rightElevator.setPower(0.4);
    }

    public void moveDown() {
        leftElevator.setPower(-0.4);
        rightElevator.setPower(-0.4);
    }

    public void stop() {
        leftElevator.setPower(0);
        rightElevator.setPower(0);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
*//*


package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ElevatorSubsystem {
    private static DcMotor leftElevator;
    private static DcMotor rightElevator;
    private final PID elevatorPID;

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");

        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorPID = new PID(0.01, 0.0, 0.0);
    }

    public void moveUp() {
        leftElevator.setPower(0.4);
        rightElevator.setPower(0.4);
    }

    public void moveDown() {
        leftElevator.setPower(-0.4);
        rightElevator.setPower(-0.4);
    }

    public void stop() {
        leftElevator.setPower(0);
        rightElevator.setPower(0);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setElevatorTarget(int targetTicks) {
        elevatorPID.setSetPoint(targetTicks);
    }

    public void updatePIDControl() {
        double power = elevatorPID.updatePID(leftElevator.getCurrentPosition());
        power = Math.min(Math.max(power, -1.0), 1.0);
        leftElevator.setPower(power);
        rightElevator.setPower(power);
    }

    public boolean isAtTarget() {
        return Math.abs(elevatorPID.getError()) <= 10;
    }

    public static int getCurrentLeftPosition() {
        return leftElevator.getCurrentPosition();

    }
    public static int getCurrentRightPosition() {
        return rightElevator.getCurrentPosition();

    }
}


//make a run function

//public static void run

//pid.updatePID(getENcoderPosition)

//setPower(pid.result)




*/
/*
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ElevatorSubsystem {
    public static DcMotor leftElevator;
    private static DcMotor pivot;
    public static DcMotor rightElevator;
    private static PID elevatorPID;

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");
        pivot = hardwareMap.get(DcMotor.class,"pivot");
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorPID = new PID(0.01, 0.0, 0.0);
    }

    public static void setTargetTicks(int targetTicks) {
        elevatorPID.setSetPoint(targetTicks);
    }
public static void up(){
        leftElevator.setPower(0.1);
        rightElevator.setPower(0.1);
}

public static void down(){
        leftElevator.setPower(-0.1);
        rightElevator.setPower(-0.1);
}

    public static void run() {
        double power = elevatorPID.updatePID(getCurrentLeftPosition());
        leftElevator.setPower(power);
        rightElevator.setPower(power);
    }
    public static void stop() {
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftElevator.setPower(0);
        rightElevator.setPower(0);
    }
    public static int getCurrentLeftPosition() {
        return leftElevator.getCurrentPosition();
    }
public static int getCurrentPivotPosition(){return pivot.getCurrentPosition();
}
public static int getCurrentRightPosition() {
        return rightElevator.getCurrentPosition();
    }
}
*/
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.PID;

public class Elevator extends Subsystem {


    private static final PID leftPid = new PID(0.3, 0.0, 0.0);
    private static final PID rightPid = new PID(0.3, 0.0, 0.0);


    private static DcMotor leftElevator;
    private static DcMotor rightElevator;


    public static void initialize(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");

        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private static void setLeftPower(double power) {
        leftElevator.setPower(power);
    }

    private static void setRightPower(double power) {
        rightElevator.setPower(power);
    }


    public static void stop() {
        setLeftPower(0);
        setRightPower(0);
    }

    // Resets the encoders for both elevator motors
    public static void resetEncoders() {
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public static int getLeftEncoderPosition() {
        return leftElevator.getCurrentPosition();
    }


    public static int getRightEncoderPosition() {
        return rightElevator.getCurrentPosition();
    }


    public static void runUsingPID(double leftTarget, double rightTarget) {
        leftPid.setSetPoint(leftTarget);
        rightPid.setSetPoint(rightTarget);
    }


    public static void run() {

        double leftCorrection = leftPid.updatePID(getLeftEncoderPosition());
        double rightCorrection = rightPid.updatePID(getRightEncoderPosition());

        setLeftPower(leftCorrection);
        setRightPower(rightCorrection);
    }


    public static void extendUp() {
        setLeftPower(1.0);
        setRightPower(1.0);
    }


    public static void extendDown() {
        setLeftPower(-1.0);
        setRightPower(-1.0);
    }
}
