package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class NewWristCommandDown implements Command{
    NewWristSubsystem wristSubsystem;

    public NewWristCommandDown(NewWristSubsystem elevator) {
        this.wristSubsystem = elevator;
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
