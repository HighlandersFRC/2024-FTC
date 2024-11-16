package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class MoveToPosition implements Command {

    private final double targetX;
    private final double targetY;
    private final double targetTheta;

    private static final double POSITION_TOLERANCE = 0.003;
    private static final double ANGLE_TOLERANCE = Math.toRadians(2);

    private final PID xPID = new PID(96, 0, 0);
    private final PID yPID = new PID(96, 0.0, 0);
    private final PID thetaPID = new PID(3.5, 0.0, 0.0);

    public MoveToPosition(double x, double y, double theta) {
        this.targetX = x;
        this.targetY = y;
        this.targetTheta = theta;

        xPID.clamp(0.3);

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
        double currentY = FinalPose.y;
        double currentTheta = Math.toRadians(-FinalPose.Yaw);

        xPID.updatePID(currentX);
        yPID.updatePID(currentY);
        thetaPID.updatePID(currentTheta);

        double xVel = xPID.getResult() * 1;
        double yVel = yPID.getResult() * 1;
        double thetaVel = Math.abs(targetTheta - currentTheta) < Math.toRadians(2)
                ? thetaPID.getResult() * 0.5 : thetaPID.getResult();

        Drive.autoDrive(new Vector(xVel, yVel), thetaVel);
    }

    @Override
    public void end() {
    }

    @Override
    public boolean isFinished() {
        double currentX = -FinalPose.x;
        double currentY = FinalPose.y;
        double currentTheta = Math.toRadians(-FinalPose.Yaw);

        return hasReachedPosition(currentX, currentY) && hasReachedAngle(currentTheta);
    }

    private boolean hasReachedPosition(double currentX, double currentY) {
        return Math.abs(targetX - currentX) < POSITION_TOLERANCE &&
                Math.abs(targetY - currentY) < POSITION_TOLERANCE;
    }

    private boolean hasReachedAngle(double currentTheta) {
        return Math.abs(targetTheta - currentTheta) < ANGLE_TOLERANCE;
    }
}
