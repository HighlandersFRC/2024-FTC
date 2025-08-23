package org.firstinspires.ftc.teamcode.Subsystems;

public class Superstructure extends Subsystem{
    private SuperState wantedSuperState = SuperState.IDLE;
    private SuperState currentSuperState = SuperState.IDLE;

    public Superstructure(String name) {
        super(name);
    }
    public enum SuperState{
        DEFAULT,
        IDLE
    }
    private void applyStates(){
        switch (currentSuperState){
            case DEFAULT:
                break;
            case IDLE:
                break;
    }}

private void handleDefaultState(){

        }
   private void handleIdleState() {
   }

   private SuperState handleStateTransitions() {
       switch (wantedSuperState) {
           case DEFAULT:
               currentSuperState = SuperState.DEFAULT;
               break;


       }
       return currentSuperState;
   }

    @Override
    public void periodic() {
        currentSuperState = handleStateTransitions();
    applyStates();
    }
}