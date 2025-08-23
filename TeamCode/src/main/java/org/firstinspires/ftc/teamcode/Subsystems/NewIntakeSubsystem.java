package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NewIntakeSubsystem extends Subsystem {
    Gamepad gamepad;
    public Servo RightIntake;
    public Servo LeftIntake;
    private NewIntakeSubsystem.INTAKE_STATE wantedSuperState = NewIntakeSubsystem.INTAKE_STATE.IDLE;
    private NewIntakeSubsystem.INTAKE_STATE currentSuperState = NewIntakeSubsystem.INTAKE_STATE.IDLE;
    public NewIntakeSubsystem(String name, Gamepad gamepad2) {
        super(name);
        this.gamepad = gamepad2;
    }

    public void init(HardwareMap hardwareMap) {
       RightIntake = hardwareMap.servo.get("IntakeRight");
       LeftIntake = hardwareMap.servo.get("IntakeLeft");
    }

    public void setWantedState(INTAKE_STATE intakeState){
        wantedSuperState = intakeState;
    }

    public enum INTAKE_STATE {
        DEFAULT,
        IDLE,
        INTAKE,
        OUTTAKE
    }

    private INTAKE_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = NewIntakeSubsystem.INTAKE_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = NewIntakeSubsystem.INTAKE_STATE.IDLE;
                break;
            case OUTTAKE:
                currentSuperState = NewIntakeSubsystem.INTAKE_STATE.INTAKE;
                break;
            case INTAKE:
                currentSuperState = NewIntakeSubsystem.INTAKE_STATE.OUTTAKE;
                break;

        }
        return currentSuperState;
    }

    private void handleDefaultState(){

    }
    private void handleIdleState() {

    }

    private void handleIntakeState() {
        RightIntake.setPosition(0.7);
        LeftIntake.setPosition(0.7);
    }

    private void handleOuttakeState() {
        RightIntake.setPosition(0.3);
        LeftIntake.setPosition(0.3);
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
            case INTAKE:
                handleIntakeState();
                break;
            case OUTTAKE:
                handleOuttakeState();
                break;

        }
    }
}
