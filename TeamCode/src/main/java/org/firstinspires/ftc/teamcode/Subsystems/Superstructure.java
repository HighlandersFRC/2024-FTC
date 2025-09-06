package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

public class Superstructure extends Subsystem {
    NewArmSubsystem armSubsystem = new NewArmSubsystem("arm");
    NewIntakeSubsystem intakeSubsystem = new NewIntakeSubsystem("intake");
    NewWristSubsystem wristSubsystem = new NewWristSubsystem("wrist");
    NewElevatorSubsystem elevatorSubsystem = new NewElevatorSubsystem("elevator");


    private Superstructure.SUPER_STATE wantedSuperState = Superstructure.SUPER_STATE.IDLE;
    private Superstructure.SUPER_STATE currentSuperState = Superstructure.SUPER_STATE.IDLE;

    public Superstructure(String name) {
        super(name);
    }

    public void setWantedState(Superstructure.SUPER_STATE superState){
        wantedSuperState = superState;
    }

    public void init(HardwareMap hardwareMap) {
        elevatorSubsystem.init(hardwareMap);
        intakeSubsystem.init(hardwareMap);
        wristSubsystem.init(hardwareMap);
        armSubsystem.init(hardwareMap);
    }


    public enum SUPER_STATE {
        DEFAULT,
        DEFAULT_ARM,
        DEFAULT_ELEVATOR,
        IDLE,
        ARM_UP,
        ARM_DOWN,
        ARM_FULLY_UP,
        ARM_FULLY_DOWN,
        ARM_SPECIMEN,
        ARM_HIGH_BUCKET,
        WRIST_UP,
        WRIST_DOWN,
        INTAKE,
        OUTTAKE,
        ELEVATOR_EXTEND,
        ELEVATOR_RETRACT
    }

    private SUPER_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = SUPER_STATE.DEFAULT;
                break;
            case DEFAULT_ARM:
                currentSuperState = SUPER_STATE.DEFAULT_ARM;
                break;
            case DEFAULT_ELEVATOR:
                currentSuperState = SUPER_STATE.DEFAULT_ELEVATOR;
                break;
            case IDLE:
                currentSuperState = SUPER_STATE.IDLE;
                break;
            case ARM_UP:
                currentSuperState = SUPER_STATE.ARM_UP;
                break;
            case ARM_DOWN:
                currentSuperState = SUPER_STATE.ARM_DOWN;
                break;
            case ARM_FULLY_UP:
                currentSuperState = SUPER_STATE.ARM_FULLY_UP;
                break;
            case ARM_FULLY_DOWN:
                currentSuperState = SUPER_STATE.ARM_FULLY_DOWN;
                break;
            case ARM_SPECIMEN:
                currentSuperState = SUPER_STATE.ARM_SPECIMEN;
                break;
            case ARM_HIGH_BUCKET:
                currentSuperState = SUPER_STATE.ARM_HIGH_BUCKET;
                break;
            case WRIST_UP:
                currentSuperState = SUPER_STATE.WRIST_UP;
                break;
            case WRIST_DOWN:
                currentSuperState = SUPER_STATE.WRIST_DOWN;
                break;
            case INTAKE:
                currentSuperState = SUPER_STATE.INTAKE;
                break;
            case OUTTAKE:
                currentSuperState = SUPER_STATE.OUTTAKE;
                break;
            case ELEVATOR_EXTEND:
                currentSuperState = SUPER_STATE.ELEVATOR_EXTEND;
                break;
            case ELEVATOR_RETRACT:
                currentSuperState = SUPER_STATE.ELEVATOR_RETRACT;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.IDLE);
        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.DEFAULT);
    }

    private void handleDefaultArmState() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.DEFAULT);
    }

    private void handleDefaultElevatorState() {
        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.DEFAULT);
    }
    private void handleIdleState() {

    }


    private void handleArmFullyUpState() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_UP);
    }

    private void handleArmFullyDownState() {
       armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_DOWN);
    }

    private void handleSpecimenState() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.SPECIMEN);
    }

    private void handleHighBucketState() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.HIGH_BUCKET);
    }

    private void handleWristUpState() {
        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_UP);
    }

    private void handleWristDownState() {
        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_DOWN);
    }

    private void handleElevatorExtendState() {
        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_EXTEND);
    }

    private void handleElevatorRetractState() {
        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.ELEVATOR_RETRACT);
    }

    private void handleIntakeState() {
        intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.INTAKE);
    }

    private void handleOuttakeState() {
        intakeSubsystem.setWantedState(NewIntakeSubsystem.INTAKE_STATE.OUTTAKE);
    }

    @Override
    public void periodic() {
        armSubsystem.periodic();
        elevatorSubsystem.periodic();
        intakeSubsystem.periodic();
        wristSubsystem.periodic();
        handleStateTransitions();
        switch (currentSuperState) {
            case DEFAULT:
                handleDefaultState();
                break;
            case DEFAULT_ARM:
                handleDefaultArmState();
                break;
            case DEFAULT_ELEVATOR:
                handleDefaultElevatorState();
                break;
            case IDLE:
                handleIdleState();
                break;
            case ARM_FULLY_UP:
                handleArmFullyUpState();
                break;
            case ARM_FULLY_DOWN:
                handleArmFullyDownState();
                break;
            case ARM_SPECIMEN:
                handleSpecimenState();
                break;
            case ARM_HIGH_BUCKET:
                handleHighBucketState();
                break;
            case WRIST_UP:
                handleWristUpState();
                break;
            case WRIST_DOWN:
                handleWristDownState();
                break;
            case INTAKE:
                handleIntakeState();
                break;
            case OUTTAKE:
                handleOuttakeState();
                break;
            case ELEVATOR_EXTEND:
                handleElevatorExtendState();
                break;
            case ELEVATOR_RETRACT:
                handleElevatorRetractState();
                break;
        }
    }
}
