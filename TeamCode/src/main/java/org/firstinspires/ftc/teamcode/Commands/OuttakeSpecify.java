package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class OuttakeSpecify implements Command {

    private final Intake intakeSubsystem;
    private final long timeout; // Duration in milliseconds
    private long endTime; // Time when the command should stop
    private double LeftPower, RightPower;

    public OuttakeSpecify(Intake intake, long timeoutMilliseconds, double leftPower, double rightPower) {
        this.intakeSubsystem = intake;
        this.timeout = timeoutMilliseconds;

        this.LeftPower = leftPower;
        this.RightPower = rightPower;
    }

    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {
        // Calculate end time using the current system time and the timeout
        endTime = System.currentTimeMillis() + timeout;
    }

    @Override
    public void execute() {
        intakeSubsystem.outtakePower(LeftPower, RightPower);
    }

    @Override
    public void end() {
        // Stop the intake when the command ends
        Intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        // Check if the current time has reached or exceeded the end time
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeSubsystem;
    }
}
