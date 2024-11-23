package org.firstinspires.ftc.teamcode.Commands;

import org.json.JSONException;

public interface Command {
    void start();
    void execute() ;
    void end();
    boolean isFinished();
}