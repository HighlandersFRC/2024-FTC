package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;

import org.firstinspires.ftc.teamcode.Tools.PID;

public class Drive extends SequentialCommandGroup {

    private final PID yawPID = new PID(0.03, 0.0, 0.0);
    private final PID drivePID = new PID(0.03, 0.0, 0.0);

    private final HardwareMap hardwareMap;

    private double speed;
    private final double distance;
    private double targetPos;
    private double currentPos;
    private final double tolerance = 0.035;

    public Drive(HardwareMap hardwareMap, double speed, double distance, CommandScheduler scheduler) {

        super(scheduler);
        this.hardwareMap = hardwareMap;
        this.speed = speed;
        this.distance = distance;

        yawPID.setSetPoint(0);
        drivePID.setSetPoint(distance);

        yawPID.setMaxInput(180);
        yawPID.setMinInput(-180);
        yawPID.setContinuous(true);
        yawPID.setMinOutput(-0.25);
        yawPID.setMaxOutput(0.25);

    }

    public String getSubsystem() {
        return "DriveTrain";
    }

    @Override
    public void start() {




        Peripherals.resetYaw();

        targetPos = distance;
        drivePID.setSetPoint(targetPos);

    }

    @Override
    public void execute() {

      FieldOfMerit.processTags();
        double currentXPos = FinalPose.y;


   drivePID.updatePID(currentXPos);

        currentPos = Peripherals.getYawDegrees();
        yawPID.updatePID(currentPos);

        double correction = -yawPID.getResult();

        double rightFrontPower = Math.max(-1, Math.min(1, speed + correction));
        double leftFrontPower = Math.max(-1, Math.min(1, speed - correction));
        double rightBackPower = -Math.max(-1, Math.min(1, speed + correction));
        double leftBackPower = Math.max(-1, Math.min(1, speed - correction));

        org.firstinspires.ftc.teamcode.Subsystems.Drive.drive(rightFrontPower, leftFrontPower, rightBackPower, leftBackPower);

    }

    @Override
    public void end() {
        org.firstinspires.ftc.teamcode.Subsystems.Drive.drive(0, 0, 0, 0);
    }

  @Override
    public boolean isFinished() {

     double currentXPos = FinalPose.y;

 if (Math.abs(currentXPos - targetPos) <= tolerance) {
            org.firstinspires.ftc.teamcode.Subsystems.Drive.stop();
            return true;

   }
        return false;
    }

}





