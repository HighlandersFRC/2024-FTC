package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class Turn implements Command {

    private double targetAngle;
    private final double angleTolerance = 0.1;
    private PID yawPID;

    public Turn(double targetAngle) {
        this.targetAngle = targetAngle;
        yawPID = new PID(0.5, 0.0, 1);
        yawPID.setMaxInput(180);
        yawPID.setMinInput(-180);
        yawPID.setContinuous(true);
        yawPID.setMinOutput(-0.3);
        yawPID.setMaxOutput(0.3);
    }

    @Override
    public void start() {

        yawPID.setSetPoint(targetAngle);

    }

    @Override
    public void execute() {
        double currentAngle = FinalPose.Yaw;
        double correction = yawPID.updatePID(currentAngle);


        Drive.drive(correction, -correction, correction, -correction);
    }

    @Override
    public void end() {
        Drive.drive(0, 0, 0, 0); // Stop all motors
        System.out.println("Turn command ended.");
    }

    @Override
    public boolean isFinished() {

        double currentAngle = FinalPose.Yaw;
        double angleDifference = targetAngle - currentAngle;


        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Angle Difference: " + angleDifference);

        return Math.abs(angleDifference) <= angleTolerance;

    }
}
