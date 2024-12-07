package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class StopIntake implements Command  {
    public static boolean StopTheIntake;
    @Override
    public void start() {
        System.out.println("StopIntake started");
        StopTheIntake = false;
    }

    @Override
    public void execute()  {
        System.out.println("StopIntake executing");
        IntakeSubsystem.setPower(0);
        StopTheIntake = false;
    }

    @Override
    public void end() {
        IntakeSubsystem.setPower(0);
        StopTheIntake = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
