package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewArmCommandSpecimen implements Command {
    private final NewArmSubsystem armSubsystem;
    private Superstructure superstructure = new Superstructure("superStructure");
    private boolean finished = false;

    public NewArmCommandSpecimen(NewArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void start() {
        superstructure.periodic();
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.SPECIMEN);
    }

    @Override
    public void execute() {
        if (Math.abs(armSubsystem.pivot.getCurrentPosition() - DegreesToEncoderTicks(50)) < 10) {
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
