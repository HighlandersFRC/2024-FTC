package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class OuttakeCommand implements Command {

    private Intake intakeSubsystem;


    public OuttakeCommand(HardwareMap hardwareMap) {
        this.intakeSubsystem = new Intake();
        Intake.initialize(hardwareMap);
    }

    @Override
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

        return intakeSubsystem.getCorrectColor();
    }
}
