package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class OuttakeSpec implements Command {

    private final Intake intakeSubsystem;
    private long startTime;

    public OuttakeSpec(Intake intake) {
        this.intakeSubsystem = intake;

    }

    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {
        startTime=System.currentTimeMillis();
    }

    @Override
    public void execute() {
        intakeSubsystem.outtake();
    }

    @Override
    public void end() {

        Intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) >= 1500;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }
}
