package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Outtake implements Command  {
public static double pos;
    private IntakeSubsystem intakeSubsystem;

    public Outtake(IntakeSubsystem intake, double pos) {
        this.intakeSubsystem = intake;
        Outtake.pos = pos;
        this.intakeSubsystem.setPosition(pos);
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
        intakeSubsystem.setPosition(pos);
    }

    @Override
    public void end() {
        intakeSubsystem.setPosition(1);
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