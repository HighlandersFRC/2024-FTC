package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;

import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class MoveToPosition implements Command {

    private double targetX;
    private double targetY;
    private double targetTheta;

    private final double positionTolerance = 0.02;
    private final double angleTolerance = Math.toRadians(2);

    private PID xPID;
    private PID yPID;
    private PID thetaPID;

    public MoveToPosition(double x, double y, double theta) {
        this.targetX = x;
        this.targetY = y;
        this.targetTheta = theta;

        xPID = new PID(7.0, -1, 9);
        yPID = new PID(7.0, -1, 9);
        thetaPID = new PID(2.5, 0.0, 2.0);
    }

    @Override
    public void start() {
        xPID.setSetPoint(targetX);
        yPID.setSetPoint(targetY);
        thetaPID.setSetPoint(targetTheta);
    }

    @Override
public void execute() throws InterruptedException {
        FinalPose.poseUpdate();
        double currentX = -FinalPose.x;
        double currentY = -FinalPose.y;
        double currentTheta = Math.toRadians(FinalPose.Yaw);


        xPID.updatePID(currentX);
        yPID.updatePID(currentY);
        thetaPID.updatePID(currentTheta);

        double xVel = xPID.getResult() * 2;
        double yVel = yPID.getResult() * 2;
        double thetaVel = Math.abs(targetTheta - currentTheta) < Math.toRadians(5) ? thetaPID.getResult() * 0.5 : thetaPID.getResult();

        Drive.autoDrive(new Vector(xVel, yVel), thetaVel);
    }

    @Override
    public void end() {
        Drive.stop();


    }

    @Override
    public boolean isFinished() {
        double currentX = -FinalPose.x;
        double currentY = -FinalPose.y;
        double currentTheta = Math.toRadians(FinalPose.Yaw);

        boolean positionReached = Math.abs(targetX - currentX) < positionTolerance
                && Math.abs(targetY - currentY) < positionTolerance;
        boolean angleReached = Math.abs(targetTheta - currentTheta) < angleTolerance;

        return positionReached && angleReached;
    }
}
