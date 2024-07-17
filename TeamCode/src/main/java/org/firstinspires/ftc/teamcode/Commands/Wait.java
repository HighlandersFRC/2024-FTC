package org.firstinspires.ftc.teamcode.Commands;

public class Wait implements Command {
    private final long duration;
    private long startTime;
    private boolean isStarted;

    public Wait(long duration) {
        this.duration = duration;
        this.isStarted = false;
    }

    @Override
    public String getSubsystem() {
        return "Misc";
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        isStarted = true;
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - startTime;
        if (elapsedTime % 1000 == 0) {
            System.out.println("Wait command: " + elapsedTime / 1000 + " seconds elapsed");
        }
    }

    @Override
    public void end() {
        System.out.println("Wait Ended");
        // Reset all internal state variables
        startTime = 0;
        isStarted = false;
    }

    @Override
    public boolean isFinished() {
        if (!isStarted) {
            start();
        }
        return System.currentTimeMillis() - startTime >= duration;
    }
}
