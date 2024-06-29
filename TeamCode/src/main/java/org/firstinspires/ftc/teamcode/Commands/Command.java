package org.firstinspires.ftc.teamcode.Commands;

public interface Command {

    void start();
    void execute();

    void end();

    boolean isFinished();

    String getSubsystem();
}
