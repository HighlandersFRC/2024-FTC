package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.Commands.StopIntake.StopTheIntake;

public class Intake implements Command  {
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
}
