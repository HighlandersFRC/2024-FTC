package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class WristMove implements Command {

    String name = "Wrist";

    public WristMove(double pos){
        Wrist.move(pos);
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
        return true;
    }
}
