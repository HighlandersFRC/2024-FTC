package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Outtake implements Command  {
    private IntakeSubsystem intakeSubsystem;

    public Outtake(IntakeSubsystem outtake) {
        this.intakeSubsystem = outtake;
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
        intakeSubsystem.setPosition(0.75);
    }

    @Override
    public void end() {
        intakeSubsystem.setPosition(0.75);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getPositionLeft() == 0.25 && intakeSubsystem.getPositionRight() == 0.85;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }

}