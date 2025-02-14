package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MAX_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MIN_TICKS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.setPowerToPercentage;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends Subsystem {
    public DcMotor pivot;
    private double armPos = 0;
    public DigitalChannel limitSwitch;
    double power = 0;

    public ArmSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        this.pivot = null;
        initialize(hardwareMap);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private double getCurrentPosition() {
        return pivot.getCurrentPosition();
    }


    public void initialize(HardwareMap hardwareMap) {
        pivot = hardwareMap.dcMotor.get("pivotMotor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
public double getCurrentPositionWithLimitSwitch() {
        double currentPos = getCurrentPosition();
        if (limitSwitch != null && !limitSwitch.getState()) {
            currentPos = 0;
        }
        return currentPos;
}


    public void setPower(double power) {
        if (pivot != null) {
            pivot.setPower(power);
        }
    }

    public void manual(Gamepad gamepad1) {
//
//        double currentPosition = getCurrentPosition();
//        System.out.println("asxsssdf");
//        System.out.println(gamepad1.b &&!(currentPosition < -300));
//
//            if (!(currentPosition > MIN_TICKS - 100)&&gamepad1.a) {
//                System.out.println("First Passed");
//
//                    System.out.println("Second Passed");
//                    power = setPowerToPercentage(100);
//                    setPower(setPowerToPercentage(100));
//
//            } else if (gamepad1.b && !(currentPosition < -300)) {
//                System.out.println("b");
//                power = setPowerToPercentage(-100);
//                setPower(setPowerToPercentage(-100));
//
//            } else {
//                System.out.println("else Statement");
//                Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                setPower(0);
//            }

        if (gamepad1.left_bumper){
            setPower(0.8);
        } else if (gamepad1.right_bumper) {
            setPower(-0.8);
        } else {
            setPower(0);
        }

    }

    public void setPosition(double pos) {
        if (pivot != null) {
            pivotPID.setSetPoint(pos);
            pivotPID.updatePID(getCurrentPositionWithLimitSwitch());
            pivotPID.setMaxOutput(0.3);
            pivotPID.setMinOutput(-0.3);
            pivot.setPower(pivotPID.getResult());
        }
    }



    public void contolArm(Gamepad gamepad1) {

            if (gamepad1.y) {
                armPos = DegreesToEncoderTicks(120);
            } else if (gamepad1.b) {
                armPos = DegreesToEncoderTicks(0);
            } else if (gamepad1.touchpad) {
                armPos = DegreesToEncoderTicks(90);
            } else if (gamepad1.a) {
               armPos = DegreesToEncoderTicks(35);
            } else if (gamepad1.x) {
                armPos = DegreesToEncoderTicks(70);
            }

            pivotPID.setSetPoint(armPos);

            pivotPID.updatePID(getCurrentPositionWithLimitSwitch());
            pivotPID.setMaxOutput(1);
            pivotPID.setMinOutput(-1);
            pivot.setPower(pivotPID.getResult());
        }

    }