package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class OuttakeCommand implements Command {

    private Intake intakeSubsystem;


    public OuttakeCommand() {
        this.intakeSubsystem = new Intake();
    }

    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {
        intakeSubsystem.stopIntake();
    }

    @Override
    public void execute() {

        intakeSubsystem.outtake();
    }

    @Override
    public void end() {

        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {

        return true;
    }
}