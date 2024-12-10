package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class WristCommands implements Command {
    public static double pos;

    public WristCommands(double tarPos) {
        pos = tarPos;
    }
    @Override
    public void start() {
        System.out.println("Wrist Command Started");
    }

    @Override
    public void execute() {
        Wrist.wrist.setPosition(pos);
    }

    @Override
    public void end() {
       Wrist.wrist.setPosition(0.5);
    }

    @Override
    public boolean isFinished() {
       return Wrist.getPosition() == pos;
    }
}
