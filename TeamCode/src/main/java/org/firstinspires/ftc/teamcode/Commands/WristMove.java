package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class WristMove implements Command {

    String name = "Wrist";
    Wrist wristSubsystem;
    public WristMove(Wrist wrist, double pos) {
        Wrist.move(pos);
        this.wristSubsystem = wrist;
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

    @Override
    public Subsystem getRequiredSubsystem() {
        return wristSubsystem;
    }
}
