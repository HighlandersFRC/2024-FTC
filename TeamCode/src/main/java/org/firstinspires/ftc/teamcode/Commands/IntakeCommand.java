package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class IntakeCommand implements Command {

    private Intake intakeSubsystem;


    public IntakeCommand(HardwareMap hardwareMap) {
        this.intakeSubsystem = new Intake();
        Intake.initialize(hardwareMap);
    }

    @Override
    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {


    }

    @Override
    public void execute() {

        if (!intakeSubsystem.getCorrectColor()) {
            intakeSubsystem.intake();
        } else {
            intakeSubsystem.stopIntake();
        }
    }

    @Override
    public void end() {

        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {

        return intakeSubsystem.getCorrectColor();
    }
}
