package org.firstinspires.ftc.teamcode.Commands;

public class Wait implements Command {
    public String getSubsystem() {
        return "Misc";
    }
        long time;
    long endTime;
    public Wait(long time){
                this.time = time;
    }
    public void start(){
        endTime = System.currentTimeMillis() + time;
    }


    public void initialize() {

    }

    public void execute(){

    }
    public void end(){
    }

    public boolean isFinished() {
        if (System.currentTimeMillis() >= endTime){
            return true;
        }
        else {
            return false;
        }
    }
}
