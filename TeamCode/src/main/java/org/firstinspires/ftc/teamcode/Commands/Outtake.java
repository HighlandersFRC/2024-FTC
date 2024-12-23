package org.firstinspires.ftc.teamcode.Commands;


import static org.firstinspires.ftc.teamcode.Commands.StopIntake.StopTheIntake;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Outtake implements Command  {

    IntakeSubsystem intakeSubsystem;

    public Outtake(IntakeSubsystem intake) {
        intakeSubsystem = intake;
    }

    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {
     System.out.println("Outtake started");
    }

    @Override
    public void execute()  {
    System.out.println("Outtake executing");
    IntakeSubsystem.setPower(-1);
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

