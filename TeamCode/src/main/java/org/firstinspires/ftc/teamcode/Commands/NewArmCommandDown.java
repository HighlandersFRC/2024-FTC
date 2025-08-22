package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewArmCommandDown implements Command {
    private final NewArmSubsystem armSubsystem;
    private boolean finished = false;
    private Superstructure superstructure = new Superstructure("superStructure");
    public NewArmCommandDown(NewArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void start() {
        superstructure.periodic();
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_DOWN);
    }

    @Override
    public void execute() {
        if (Math.abs(armSubsystem.pivot.getCurrentPosition() - DegreesToEncoderTicks(0)) < 10) {
            finished = true;
        }
    }

    @Override
    public void end() {
        superstructure.periodic();
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
