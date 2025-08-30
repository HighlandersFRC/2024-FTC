package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewArmCommandDown implements Command {

    NewArmSubsystem armSubsystem;
    Superstructure superstructure;
    public NewArmCommandDown(NewArmSubsystem arm, Superstructure superstructure) {
        this.armSubsystem = arm;
        this.superstructure = superstructure;
    }


    @Override
    public void start() {
        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_DOWN);
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