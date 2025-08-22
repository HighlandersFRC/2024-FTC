package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class NewArmCommandHighBucket implements Command {
    private final NewArmSubsystem armSubsystem;
    private boolean finished = false;

    public NewArmCommandHighBucket(NewArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void start() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.HIGH_BUCKET);
    }

    @Override
    public void execute() {
        if (Math.abs(armSubsystem.pivot.getCurrentPosition() - DegreesToEncoderTicks(70)) < 10) {
            finished = true;
        }
    }

    @Override
    public void end() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.IDLE);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return armSubsystem;
    }
}
