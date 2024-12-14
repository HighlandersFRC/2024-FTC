package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class OuttakeCommand implements Command {



    public OuttakeCommand() {

    }

    public String getSubsystem() {
        return "Intake";
    }

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        Intake.rightServo.setPower(-1);
        Intake.leftServo.setPower(1);
    }

    @Override
    public void end() {

        Intake.rightServo.setPower(0);
        Intake.leftServo.setPower(0);
        Intake.stopIntake();

    }

    @Override
    public boolean isFinished() {

        return false;
    }
}