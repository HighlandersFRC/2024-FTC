package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NewIntakeSubsystem extends Subsystem{

    Gamepad gamepad;
    Servo RightIntake;
    Servo LeftIntake;

    private NewIntakeSubsystem.INTAKE_STATE wantedSuperState = NewIntakeSubsystem.INTAKE_STATE.IDLE;

    private NewIntakeSubsystem.INTAKE_STATE currentSuperState = NewIntakeSubsystem.INTAKE_STATE.IDLE;

    public NewIntakeSubsystem(String name, Gamepad gamepad2) {
        super(name);
        this.gamepad = gamepad2;
    }

    public void init(HardwareMap hardwareMap) {
        RightIntake = hardwareMap.servo.get("RightIntake");
        LeftIntake = hardwareMap.servo.get("LeftIntake");
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
           case INTAKE:
               currentSuperState = NewIntakeSubsystem.INTAKE_STATE.INTAKE;
               break;
           case OUTTAKE:
               currentSuperState = NewIntakeSubsystem.INTAKE_STATE.OUTTAKE;
               break;

       }
       return currentSuperState;
   }

   private void handleDefaultState(){

   }
   private void handleIdleState() {
       RightIntake.setPosition(0.5); //UPDATE
       LeftIntake.setPosition(0.5); //UPDATE
   }

   private void handleIntakeState() {
        RightIntake.setPosition(0.5); //UPDATE
        LeftIntake.setPosition(0.5); //UPDATE
   }

   private void handleOuttakeState() {
       RightIntake.setPosition(0); //UPDATE
       LeftIntake.setPosition(0); //UPDATE
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
