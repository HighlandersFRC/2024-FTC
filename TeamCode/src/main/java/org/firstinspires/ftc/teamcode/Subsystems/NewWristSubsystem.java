package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NewWristSubsystem extends Subsystem {
    Gamepad gamepad;
    Servo wrist;
    private NewWristSubsystem.WRIST_STATE wantedSuperState = NewWristSubsystem.WRIST_STATE.IDLE;
    private NewWristSubsystem.WRIST_STATE currentSuperState = NewWristSubsystem.WRIST_STATE.IDLE;
    public NewWristSubsystem(String name, Gamepad gamepad2) {
        super(name);
        this.gamepad = gamepad2;
    }

    public void init(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist"); //UPDATE THIS
    }

    public void setWantedState(WRIST_STATE wristState){
        wantedSuperState = wristState;
    }

    public enum WRIST_STATE {
        DEFAULT,
        IDLE,
        WRIST_UP,
        WRIST_DOWN
    }

    private WRIST_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = NewWristSubsystem.WRIST_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = NewWristSubsystem.WRIST_STATE.IDLE;
                break;
            case WRIST_UP:
                currentSuperState = NewWristSubsystem.WRIST_STATE.WRIST_UP;
                break;
            case WRIST_DOWN:
                currentSuperState = NewWristSubsystem.WRIST_STATE.WRIST_DOWN;
                break;

        }
        return currentSuperState;
    }

    private void handleDefaultState(){

    }
    private void handleIdleState() {

    }

    private void handleWristDownState() {


    }

    private void handleWristUpState() {

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
            case WRIST_UP:
                handleWristUpState();
                break;
            case WRIST_DOWN:
                handleWristDownState();
                break;

        }
    }
}
