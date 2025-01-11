
package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;



public class Intake implements Command  {
public static double pos;
    IntakeSubsystem intakeSubsystem;

    public Intake(IntakeSubsystem intake, double pos) {
        this.intakeSubsystem = intake;
        Intake.pos = pos;
        this.intakeSubsystem.setPosition(pos);
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
        intakeSubsystem.setPosition(pos);
    }

    @Override
    public void end() {
        intakeSubsystem.setPosition(pos);
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
