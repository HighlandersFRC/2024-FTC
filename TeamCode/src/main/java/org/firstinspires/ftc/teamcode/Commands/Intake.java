
package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;



public class Intake implements Command  {

    IntakeSubsystem intakeSubsystem;

    public Intake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeSubsystem.setPosition(0.5);
    }

    public String getSubsystem() {
        return "Intake";
    }
    @Override
    public void start() {
        System.out.println("Intake started");
    }

    @Override
    public void execute() {
        System.out.println("Intake executing");
        intakeSubsystem.setPosition(0.5);
    }

    @Override
    public void end() {
        intakeSubsystem.setPosition(0.5);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getPositionLeft() == 1 && intakeSubsystem.getPositionRight() == -1;
    }
    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }
}
