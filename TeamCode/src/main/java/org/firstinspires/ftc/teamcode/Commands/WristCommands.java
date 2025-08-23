package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class WristCommands implements Command {
    public double pos;

    private String name = "Wrist";
    private Wrist wristSubsystem;

    public WristCommands(Wrist wristSubsystem, double pos) {
        this.wristSubsystem = wristSubsystem;
        this.pos = pos;

    }

    @Override
    public void start() {
        System.out.println("Wrist Command Started");
    }

    @Override
    public void execute() {
        wristSubsystem.setPosition(pos);
    }

    @Override
    public void end() {
        wristSubsystem.setPosition(pos);
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.getPosition() == pos;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return wristSubsystem;
    }
}