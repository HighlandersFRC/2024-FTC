package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewArmCommandUp implements Command {
    private Superstructure superstructure = new Superstructure("superStructure");
    private final NewArmSubsystem armSubsystem;
    private boolean finished = false;

    public NewArmCommandUp(NewArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void start() {
        superstructure.periodic();
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_UP);
    }

    @Override
    public void execute() {
        if (Math.abs(armSubsystem.pivot.getCurrentPosition() - DegreesToEncoderTicks(90)) < 10) {
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
