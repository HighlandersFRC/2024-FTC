package org.firstinspires.ftc.teamcode.Commands;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class Strafe extends SequentialCommandGroup {

    // PID controllers for driving and correcting yaw
    private final PID yawPID = new PID(0.03, 0.0, 0.0);
    private final PID drivePID = new PID(0.03, 0.0, 0.0);

    // Hardware and sensor references
    private final DriveTrain driveTrain = new DriveTrain();
    private SparkFunOTOS mouse;
    private final HardwareMap hardwareMap;

    // Robot state variables
    private double speed;
    private final double distance;
    private double targetPos;
    private double currentPos;

    public Strafe (HardwareMap hardwareMap, double speed, double distance) {
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
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        DriveTrain.initialize(hardwareMap);

        Peripherals.resetYaw();

        targetPos = distance;
        drivePID.setSetPoint(targetPos);
    }

    @Override
    public void execute() {
        SparkFunOTOS.Pose2D position = mouse.getPosition();
        double currentXPos = position.y;

        // Update the drive and yaw PIDs
        drivePID.updatePID(currentXPos);
        currentPos = -Peripherals.getYawDegrees();
        yawPID.updatePID(currentPos);

        // Calculate corrections from yaw PID
        double correction = yawPID.getResult();

        // Set motor powers based on speed and correction
        double rightFrontPower = speed + correction;
        double leftFrontPower = speed + correction;
        double rightBackPower = speed + correction;
        double leftBackPower = speed + correction;

        DriveTrain.drive(-rightFrontPower, leftFrontPower, rightBackPower, -leftBackPower);
    }

    @Override
    public void end() {
        DriveTrain.drive(0, 0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        SparkFunOTOS.Pose2D position = mouse.getPosition();
        double currentYPos = position.y*1.2;
        if (Math.abs(currentYPos) + 0.035 >= Math.abs(targetPos)) {
            DriveTrain.stop();
            return true;
        }
        return false;


    }
}
