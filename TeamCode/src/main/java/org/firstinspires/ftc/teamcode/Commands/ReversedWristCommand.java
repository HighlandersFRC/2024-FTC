package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class ReversedWristCommand implements Command {
    public static double pos;

    private String name = "Wrist";
    private Wrist wristSubsystem;

    public ReversedWristCommand(Wrist wristSubsystem, double pos) {
        this.wristSubsystem = wristSubsystem;
        WristCommands.pos = pos; // Set the position for the wrist

        this.wristSubsystem.setPosition(pos);
    }

    @Override
    public void start() {
        System.out.println("Wrist Command Started");
    }

    @Override
    public void execute() {
        wristSubsystem.setPosition(pos); // Use the wrist subsystem to set position
    }

    @Override
    public void end() {
        wristSubsystem.setPosition(pos); // Reset to neutral position
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.getPosition() == pos; // Compare desired and current position
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return wristSubsystem;
    }
}