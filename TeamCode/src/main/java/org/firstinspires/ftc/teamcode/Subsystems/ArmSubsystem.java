package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmSubsystem extends Subsystem {
    public static DcMotor pivotMotor;
    public static DigitalChannel limitSwitch; // Limit switch for arm position
    private static double armPosition = 0; // Target arm position
    static PID piviotPID = new PID(0.15, 0, 1);
    protected static double pos = 0;
    protected static int NumberOfTimesPressedB = 0;
    protected static int NumberOfTimesPressedStart = 0;
    // Initialize the motor and limit switch
    public static void initialize(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        // Configure the limit switch
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Set the motor to brake mode
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run without encoder for direct power control
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        piviotPID.updatePID(pivotMotor.getCurrentPosition());
        piviotPID.setMaxOutput(1);
        piviotPID.setMinOutput(-1);
        piviotPID.setSetPoint(armPosition);
        pivotMotor.setPower(piviotPID.getResult());
    }

    public static void initializeWithOutLimit(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");

        // Set the motor to brake mode
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run without encoder for direct power control
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        piviotPID.updatePID(pivotMotor.getCurrentPosition());
        piviotPID.setMaxOutput(1);
        piviotPID.setMinOutput(-1);
        piviotPID.setSetPoint(armPosition);
        pivotMotor.setPower(piviotPID.getResult());
    }

    // Set motor power
    public static void setPower(double power) {
        pivotMotor.setPower(power);
    }

    private static boolean previousBState = false;
    private static long lastProcessedTime = 0;
    private static final long minDelay = 100; // Minimum delay (in ms)
    private static final long maxDelay = 500; // Maximum delay (in ms)

    private static final boolean previousStartState = false;
    private static long lastStartTime = 0;
    private static final long min_delay = 100; // Minimum delay (in ms)
    private static final long max_delay = 500; // Maximum delay (in ms)
//Underscore is for changing options
    // Integrate the logic into the controlPivot method
    public static void controlPivot(Gamepad gamepad1, PID pivotPID) {
        long process_delay = min_delay + (long) (Math.random() * (max_delay - min_delay + 1));

        boolean currentStartState = gamepad1.start;


        long currentScedule = System.currentTimeMillis();


        if (currentStartState && !previousStartState && (currentScedule - lastStartTime > process_delay)) {
            lastStartTime = currentScedule;

            NumberOfTimesPressedStart++;
            System.out.println(NumberOfTimesPressedStart + "here");

        }

            if (NumberOfTimesPressedStart == 2) {
                NumberOfTimesPressedStart = 0;
            }

            if (NumberOfTimesPressedStart == 1) {

                // Calculate a random delay between minDelay and maxDelay
                long processDelay = minDelay + (long) (Math.random() * (maxDelay - minDelay + 1));

                // Get the current state of button B
                boolean currentBState = gamepad1.b;

                // Get the current time in milliseconds
                long currentTime = System.currentTimeMillis();

                // Check if button B has just been pressed and process within the dynamic delay range
                if (currentBState && !previousBState && (currentTime - lastProcessedTime > processDelay)) {
                    // Update the last processed time to the current time
                    lastProcessedTime = currentTime;

                    // Increment the press count
                    NumberOfTimesPressedB++;
                    System.out.println("Button B Pressed! Count: " + NumberOfTimesPressedB);

                    // Update arm position based on the number of presses
                    switch (NumberOfTimesPressedB) {
                        case 0:
                            armPosition = 0;
                            System.out.println("Case 0 armPosition: " + armPosition);
                            break;
                        case 1:
                            armPosition = -3176;
                            System.out.println("Case 1 armPosition: " + armPosition);
                            break;
                        case 2:
                            armPosition = -3550;
                            System.out.println("Case 2 armPosition: " + armPosition);
                            break;
                        case 3:
                            armPosition = -3176;
                            System.out.println("Case 3 armPosition: " + armPosition);
                            break;
                        case 4:
                            armPosition = 0;
                            System.out.println("Case 4 armPosition: " + armPosition);
                            // Reset press count to 0 after 4 presses
                            NumberOfTimesPressedB = 0;
                            break;
                    }
                }

                // Update the previous state of button B for the next iteration
                previousBState = currentBState;


            }
        if (gamepad1.y&&NumberOfTimesPressedStart==0) {
            armPosition = -3176;
        } else if (gamepad1.b&&NumberOfTimesPressedStart==0) {
            armPosition = 0;
        } else if (gamepad1.x&&NumberOfTimesPressedStart==0) {
            armPosition = -3550;
        }

        if (gamepad1.a) {
            armPosition=-1736;
        }
        // PID control logic for pivoting
        pivotPID.setSetPoint(armPosition);
        pivotPID.setMaxOutput(1);
        pivotPID.setMinOutput(-1);
        pivotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());

        // Set motor power using the PID result
        pivotMotor.setPower(-pivotPID.getResult());





    }
    // Control pivot motor using only gamepad inputs
    public static void controlPivotWithoutLimitSwitch(Gamepad gamepad1, PID pivotPID ) {
        // Control the motor based on gamepad input
        pivotPID.setSetPoint(armPosition);
        pivotPID.setMaxOutput(1);
        pivotPID.setMinOutput(-1);
        pivotPID.updatePID(pivotMotor.getCurrentPosition());

        // Set motor power using the PID result
        pivotMotor.setPower(-pivotPID.getResult());

        // Allow manual override for fine control

    }


    // Get the current position of the motor in encoder ticks
    public static int getCurrentPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public static double getCurrentPositionWithLimitSwitch() {

        if(!limitSwitch.getState()){
            pos = ArmSubsystem.getCurrentPosition();;


        }
        return ArmSubsystem.getCurrentPosition() - pos;
    }

    public static boolean limitSwitch() {

        return limitSwitch.getState();
    }

    // Get the current target arm position
    public static double getTargetPosition() {
        return armPosition;
    }
}

