package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NewArmSubsystem extends Subsystem {
Gamepad gamepad;
private DigitalChannel limitSwitch;
public DcMotor pivot;
private double pos;
    private NewArmSubsystem.ARM_STATE wantedSuperState = NewArmSubsystem.ARM_STATE.IDLE;
    private NewArmSubsystem.ARM_STATE currentSuperState = NewArmSubsystem.ARM_STATE.IDLE;
    public NewArmSubsystem(String name, Gamepad gamepad2) {
        super(name);
        this.gamepad = gamepad2;
    }

    public void init(HardwareMap hardwareMap) {
        pivot = hardwareMap.dcMotor.get("pivotMotor");
        limitSwitch = hardwareMap.digitalChannel.get("limitSwitch");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double getCurrentPositionWithLimitSwitch() {
        double currentPos = pivot.getCurrentPosition();
        if (limitSwitch != null && !limitSwitch.getState()) {
            currentPos = 0;
        }
        return currentPos;
    }
    public void setWantedState(ARM_STATE armState){
        wantedSuperState = armState;
    }

    public enum ARM_STATE {
        DEFAULT,
        IDLE,
        ARM_UP,
        ARM_DOWN,
        ARM_FULLY_UP,
        ARM_FULLY_DOWN,
        SPECIMEN,
        HIGH_BUCKET
    }

    private ARM_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = NewArmSubsystem.ARM_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = NewArmSubsystem.ARM_STATE.IDLE;
                break;
            case ARM_UP:
                currentSuperState = NewArmSubsystem.ARM_STATE.ARM_UP;
                break;
            case ARM_DOWN:
                currentSuperState = NewArmSubsystem.ARM_STATE.ARM_DOWN;
                break;
            case ARM_FULLY_UP:
                currentSuperState = NewArmSubsystem.ARM_STATE.ARM_FULLY_UP;
                break;
            case ARM_FULLY_DOWN:
                currentSuperState = NewArmSubsystem.ARM_STATE.ARM_FULLY_DOWN;
                break;
            case SPECIMEN:
                currentSuperState = NewArmSubsystem.ARM_STATE.SPECIMEN;
                break;
            case HIGH_BUCKET:
                currentSuperState = NewArmSubsystem.ARM_STATE.HIGH_BUCKET;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState(){
    }
    private void handleIdleState() {
        pivotPID.setSetPoint(pos);
        pivotPID.updatePID(getCurrentPositionWithLimitSwitch());
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinOutput(-0.5);
        pivot.setPower(-pivotPID.getResult());
    }

    private void handleArmUpState() {
        pivot.setPower(-0.5);
    }

    private void handleArmDownState() {
            pivot.setPower(0.5);
    }



    private void handleArmFullyUpState() {
       pos = DegreesToEncoderTicks(90);
        pivotPID.setSetPoint(pos);
        pivotPID.updatePID(getCurrentPositionWithLimitSwitch());
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinOutput(-0.5);
        pivot.setPower(-pivotPID.getResult());
    }

    private void handleArmFullyDownState() {
        pos = DegreesToEncoderTicks(0);
        pivotPID.setSetPoint(pos);
        pivotPID.updatePID(getCurrentPositionWithLimitSwitch());
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinOutput(-0.5);
        pivot.setPower(-pivotPID.getResult());
    }

    private void handleSpecimenState() {
        pos = DegreesToEncoderTicks(50);
        pivotPID.setSetPoint(pos);
        pivotPID.updatePID(getCurrentPositionWithLimitSwitch());
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinOutput(-0.5);
        pivot.setPower(-pivotPID.getResult());
    }

    private void handleHighBucketState() {
        pos = DegreesToEncoderTicks(70);
        pivotPID.setSetPoint(pos);
        pivotPID.updatePID(getCurrentPositionWithLimitSwitch());
        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinOutput(-0.5);
        pivot.setPower(-pivotPID.getResult());
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        switch (currentSuperState) {
            case DEFAULT:
                handleDefaultState();
                break;
            case IDLE:
                handleIdleState();
                break;
            case ARM_UP:
                handleArmUpState();
                break;
            case ARM_DOWN:
                handleArmDownState();
                break;
            case ARM_FULLY_UP:
                handleArmFullyUpState();
                break;
            case ARM_FULLY_DOWN:
                handleArmFullyDownState();
                break;
            case SPECIMEN:
                handleSpecimenState();
                break;
            case HIGH_BUCKET:
                handleHighBucketState();
                break;
        }
    }
}
