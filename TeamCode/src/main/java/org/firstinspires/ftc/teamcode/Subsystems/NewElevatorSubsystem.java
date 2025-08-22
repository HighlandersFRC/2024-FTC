package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NewElevatorSubsystem extends Subsystem {
    Gamepad gamepad;
    DcMotor elevator;
    private NewElevatorSubsystem.ELEVATOR_STATE wantedSuperState = NewElevatorSubsystem.ELEVATOR_STATE.IDLE;
    private NewElevatorSubsystem.ELEVATOR_STATE currentSuperState = NewElevatorSubsystem.ELEVATOR_STATE.IDLE;
    public NewElevatorSubsystem(String name, Gamepad gamepad2) {
        super(name);
        this.gamepad = gamepad2;
    }

    public void init(HardwareMap hardwareMap) {
        elevator = hardwareMap.dcMotor.get(""); //UPDATE THIS
    }

    public void setWantedState(ELEVATOR_STATE elevatorState){
        wantedSuperState = elevatorState;
    }

    public enum ELEVATOR_STATE {
        DEFAULT,
        IDLE,
        ELEVATOR_EXTEND,
        ELEVATOR_RETRACT
    }

    private ELEVATOR_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = NewElevatorSubsystem.ELEVATOR_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = NewElevatorSubsystem.ELEVATOR_STATE.IDLE;
                break;
            case ELEVATOR_EXTEND:
                currentSuperState = NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_EXTEND;
                break;
            case ELEVATOR_RETRACT:
                currentSuperState = NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_RETRACT;
                break;

        }
        return currentSuperState;
    }

    private void handleDefaultState(){

    }
    private void handleIdleState() {
        elevator.setPower(0);
    }

    private void handleExtendState() {
        elevator.setPower(-0.5);

    }

    private void handleRetractState() {
        elevator.setPower(0.5);
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
            case ELEVATOR_EXTEND:
                handleExtendState();
                break;
            case ELEVATOR_RETRACT:
                handleRetractState();
                break;

        }
    }
}
