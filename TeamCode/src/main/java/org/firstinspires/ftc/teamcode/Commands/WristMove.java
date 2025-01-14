package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class WristMove implements Command {

    String name = "Wrist";
    double setPos1;
    double setPos2;
    Wrist wristSubsystem;
    public WristMove(Wrist wrist, double pos1, double pos2) {
        Wrist.move1(pos1);
        Wrist.move2(pos2);
        this.wristSubsystem = wrist;
        Robot.CURRENT_WRIST1 = pos1;
        Robot.CURRENT_WRIST2 = pos2;
        setPos1 = pos1;
        setPos2 = pos2;
    }

    @Override
    public void start() {
        Robot.CURRENT_WRIST1 = setPos1;
        Robot.CURRENT_WRIST2=setPos2;
    }

    @Override
    public void execute() {
        Robot.CURRENT_WRIST1 = setPos1;
        Robot.CURRENT_WRIST2 =setPos2;
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
