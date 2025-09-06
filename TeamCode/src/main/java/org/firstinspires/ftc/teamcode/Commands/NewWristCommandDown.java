package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewWristCommandDown implements Command{
    NewWristSubsystem wristSubsystem;
    Superstructure superstructure;
    public NewWristCommandDown(NewWristSubsystem elevator, Superstructure superstructure) {
        this.wristSubsystem = elevator;
        this.superstructure = superstructure;
    }

    @Override
    public void start() {
        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_DOWN);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return wristSubsystem;
    }
}
