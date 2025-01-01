package org.firstinspires.ftc.teamcode.Commands;


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
        intakeSubsystem.setPosition(1,-1);
    }

    @Override
    public void end() {
        intakeSubsystem.setPosition(1,-1);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getPositionLeft() == -1 && intakeSubsystem.getPositionRight() == 1;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }

}