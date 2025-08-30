package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class NewArmCommandHighBucket implements Command {

    NewArmSubsystem armSubsystem;
    public NewArmCommandHighBucket(NewArmSubsystem arm) {
        this.armSubsystem = arm;
    }


    @Override
    public void start() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.HIGH_BUCKET);
    }


    @Override
    public void execute() {

    }

    @Override
    public void end() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.DEFAULT);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return armSubsystem;
    }
}