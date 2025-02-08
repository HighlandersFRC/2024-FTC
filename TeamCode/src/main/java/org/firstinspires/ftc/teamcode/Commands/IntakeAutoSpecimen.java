package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class IntakeAutoSpecimen implements Command {

    private final Intake intakeSubsystem;
    private long startTime;

    public IntakeAutoSpecimen(Intake intake) {
        this.intakeSubsystem = intake;
    }

    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (!intakeSubsystem.getCorrectColor()) {
            intakeSubsystem.intake();
        } else {
            Intake.stopIntake();
        }
    }

    @Override
    public void end() {
        Intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getCorrectColor() || (System.currentTimeMillis() - startTime) >= 1750;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }
}
