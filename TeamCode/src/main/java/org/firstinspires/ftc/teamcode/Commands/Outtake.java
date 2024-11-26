package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class Outtake implements Command  {
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
        return false;
    }
}
