package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewArmCommandSpecimen implements Command {

   Superstructure superstructure;
    public NewArmCommandSpecimen(Superstructure superstructure) {
        this.superstructure = superstructure;
    }


    @Override
    public void start() {
        superstructure.setWantedState(Superstructure.SUPER_STATE.ARM_SPECIMEN);
    }


    @Override
    public void execute() {

    }

    @Override
    public void end() {
        superstructure.setWantedState(Superstructure.SUPER_STATE.DEFAULT);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return superstructure;
    }
}