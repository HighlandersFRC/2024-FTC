/*
package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class IntakeCommand implements Command {

    public enum Action {
        INTAKE,
        OUTTAKE
    }

    private final Intake intakeSubsystem;
    private final Action action;
    private boolean isFinished = false; // Tracks when the action is complete

    public IntakeCommand(HardwareMap hardwareMap, Action action) {
        this.intakeSubsystem = new Intake();
        Intake.initialize(hardwareMap);
        this.action = action;
    }

    @Override
    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {
        isFinished = false; // Reset completion state
    }

    @Override
    public void execute() {
        switch (action) {
            case INTAKE:
                if (!intakeSubsystem.getCorrectColor()) {
                    intakeSubsystem.intake();
                } else {
                    intakeSubsystem.stopIntake();
                    isFinished = true; // Mark action as complete
                }
                break;

            case OUTTAKE:
                intakeSubsystem.outtake();
                isFinished = true; // Outtake completes immediately
                break;
        }
    }

    @Override
    public void end() {
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
*/
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