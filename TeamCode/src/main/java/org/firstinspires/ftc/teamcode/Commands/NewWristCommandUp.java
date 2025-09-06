package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewWristCommandUp implements Command{
    NewWristSubsystem wristSubsystem;

    public NewWristCommandUp(NewWristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
    }

    @Override
    public void start() {
        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_UP);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_UP);

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
