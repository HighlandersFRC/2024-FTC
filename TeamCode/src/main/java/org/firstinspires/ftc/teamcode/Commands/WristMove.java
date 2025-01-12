package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class WristMove implements Command {

    String name = "Wrist";
    double setPos;
    Wrist wristSubsystem;
    public WristMove(Wrist wrist, double pos) {
        Wrist.move(pos);
        this.wristSubsystem = wrist;
        Robot.CURRENT_WRIST = pos;
        setPos = pos;
    }

    @Override
    public void start() {
        Robot.CURRENT_WRIST = setPos;
    }

    @Override
    public void execute() {
        Robot.CURRENT_WRIST = setPos;
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return wristSubsystem;
    }










}
