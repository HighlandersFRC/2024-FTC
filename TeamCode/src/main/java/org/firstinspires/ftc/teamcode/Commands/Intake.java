package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import static org.firstinspires.ftc.teamcode.Commands.StopIntake.StopTheIntake;

public class Intake implements Command  {

    IntakeSubsystem intakeSubsystem;

    public Intake(IntakeSubsystem intake) {
        intakeSubsystem = intake;
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
        IntakeSubsystem.setPower(1);
    }

    @Override
    public void end() {
        IntakeSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return StopTheIntake;
    }
 @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }
}
