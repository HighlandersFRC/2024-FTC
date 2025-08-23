
package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class IntakeCommand implements Command {


    Intake intakeSubsystem;

    public IntakeCommand(Intake intake) {
        intakeSubsystem = intake;
    }

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

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }
}
