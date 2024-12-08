package org.firstinspires.ftc.teamcode.Commands;

import org.json.JSONException;

public interface Command {
    String getSubsystem();

    void start() ;
    void execute() ;
    void end();
    boolean isFinished();
}