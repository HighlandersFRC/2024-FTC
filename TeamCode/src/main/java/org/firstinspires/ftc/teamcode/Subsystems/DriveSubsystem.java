package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends Subsystem {

    private static DcMotor leftMotor;
    private static DcMotor rightMotor;
    private static Telemetry telemetry;

    private static final double COUNTS_PER_INCH = 100; // Example value, adjust for your robot

    public DriveSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        initialize(hardwareMap);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        telemetry = new Telemetry() {
            @Override
            public Item addData(String caption, String format, Object... args) {
                return null;
            }

            @Override
            public Item addData(String caption, Object value) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, Func<T> valueProducer) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, String format, Func<T> valueProducer) {
                return null;
            }

            @Override
            public boolean removeItem(Item item) {
                return false;
            }

            @Override
            public void clear() {

            }

            @Override
            public void clearAll() {

            }

            @Override
            public Object addAction(Runnable action) {
                return null;
            }

            @Override
            public boolean removeAction(Object token) {
                return false;
            }

            @Override
            public void speak(String text) {

            }

            @Override
            public void speak(String text, String languageCode, String countryCode) {

            }

            @Override
            public boolean update() {
                return false;
            }

            @Override
            public Line addLine() {
                return null;
            }

            @Override
            public Line addLine(String lineCaption) {
                return null;
            }

            @Override
            public boolean removeLine(Line line) {
                return false;
            }

            @Override
            public boolean isAutoClear() {
                return false;
            }

            @Override
            public void setAutoClear(boolean autoClear) {

            }

            @Override
            public int getMsTransmissionInterval() {
                return 0;
            }

            @Override
            public void setMsTransmissionInterval(int msTransmissionInterval) {

            }

            @Override
            public String getItemSeparator() {
                return "";
            }

            @Override
            public void setItemSeparator(String itemSeparator) {

            }

            @Override
            public String getCaptionValueSeparator() {
                return "";
            }

            @Override
            public void setCaptionValueSeparator(String captionValueSeparator) {

            }

            @Override
            public void setDisplayFormat(DisplayFormat displayFormat) {

            }

            @Override
            public Log log() {
                return null;
            }
        }; // Initialize telemetry here or pass it from op mode

        // Set motor directions and other configurations as needed
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders if needed
        resetEncoders();

        // Set motor run modes
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToPoint(double x, double y) {
        double currentX = 0; // Placeholder for current X position, replace with actual logic
        double currentY = 0; // Placeholder for current Y position, replace with actual logic

        // Calculate angle and distance to the target point
        double angleToTarget = Math.atan2(y - currentY, x - currentX);
        double distanceToTarget = Math.sqrt(Math.pow(x - currentX, 2) + Math.pow(y - currentY, 2));

        // Convert angle to degrees
        double angleDegrees = Math.toDegrees(angleToTarget);

        // Adjust angle to fit within 0 to 360 degrees
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        // Move robot to the target point
        moveToPosition(x, y, distanceToTarget, angleDegrees);
    }

    public static void moveToPosition(double x, double y, double distance, double angleDegrees) {
        // Convert distance to encoder counts based on your robot's setup
        int targetCounts = (int) (distance * COUNTS_PER_INCH);

        // Calculate encoder targets for left and right motors
        int leftTarget = leftMotor.getCurrentPosition() + targetCounts;
        int rightTarget = rightMotor.getCurrentPosition() + targetCounts;

        // Set target positions for the motors
        leftMotor.setTargetPosition(leftTarget);
        rightMotor.setTargetPosition(rightTarget);

        // Calculate motor powers based on speed and angle
        // Example: Implement PID control or basic proportional control here
        double speed = 0.5; // Example speed
        double leftPower = speed;
        double rightPower = speed;

        // Set motor powers
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // Wait for movement to complete
        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            telemetry.addData("Status", "Moving to point...");
            telemetry.update();
        }

        // Stop the robot
        stop();
    }

    public static void stop() {
        // Stop both motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private static void resetEncoders() {
        // Reset motor encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static void setRunMode(DcMotor.RunMode mode) {
        // Set motor run mode
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }
}
