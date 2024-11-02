package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class Turn implements Command {

    private double targetAngle;
    private final double angleTolerance = 2.0; // Tolerance for angle adjustment
    private PID yawPID;

    public Turn(double targetAngle) {
        this.targetAngle = targetAngle;
        yawPID = new PID(0.5, 0.0, 0.0); // Initialize PID constants as needed
        yawPID.setMaxInput(180);
        yawPID.setMinInput(-180);
        yawPID.setContinuous(true);
        yawPID.setMinOutput(-0.3);
        yawPID.setMaxOutput(0.3);
    }

    @Override
    public void start() {
        FinalPose.Reset(); // Reset yaw to ensure accurate starting point
        yawPID.setSetPoint(targetAngle); // Set the target angle for PID
    }

    @Override
    public void execute() {
        double currentAngle = Peripherals.getYawDegrees(); // Get the current yaw angle
        double correction = -yawPID.updatePID(currentAngle); // Update PID and get correction

        // Drive the motors based on the correction for turning
        Drive.drive(correction, -correction, correction, -correction);
    }

    @Override
    public void end() {
        Drive.drive(0, 0, 0, 0); // Stop all motors
        System.out.println("Turn command ended.");
    }

    @Override
    public boolean isFinished() {
        double currentAngle = Peripherals.getYawDegrees(); // Get the current yaw angle
        // Check if the current angle is within the tolerance of the target angle
        return Math.abs(currentAngle - targetAngle) <= angleTolerance;
    }
}
