package org.firstinspires.ftc.teamcode.Commands;

public class Intake implements Command{
    public double time;
    public double startTime;

    String name = "Intake";


    public Intake(double timeInMillis, double speed){
        this.time = timeInMillis;
     }

    @Override
    public void start() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
