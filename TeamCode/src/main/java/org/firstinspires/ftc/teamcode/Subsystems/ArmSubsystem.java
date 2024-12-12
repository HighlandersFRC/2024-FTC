package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmSubsystem extends Subsystem {
    public static DcMotor pivotMotor;
    public static DigitalChannel limitSwitch; // Limit switch for arm position

    private static double armPosition = 0; // Target arm position
    protected static double pos = 0;

    // Initialize the motor and limit switch
    public static void initialize(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        // Configure the limit switch
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        Wrist.initialize(hardwareMap);
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
    public static void controlPivot(Gamepad gamepad1, PID piviotPID) {
        if (gamepad1.square) {
            piviotPID.setPID(0.015, 0, 0.01);
            armPosition = -3000;

        } else if (gamepad1.circle) {
            piviotPID.setPID(0.015, 0, 0.01);
            armPosition = 10;

        } else if (gamepad1.dpad_down) {
            piviotPID.setPID(0.015, 0, 0.01);
            armPosition = -3365;
        }

        if (gamepad1.triangle) {
            piviotPID.setPID(0.015, 0, 0.01);
            armPosition = -1936;
        }

        if (gamepad1.dpad_down) {
            Wrist.wrist.setPosition(0.62);
        }

        if(gamepad1.dpad_down) {
            IntakeSubsystem.intake.setPower(-1);
        }

        if (gamepad1.right_bumper) {
            piviotPID.setPID(0.00005, 0, 0.0000000001);
            pivotMotor.setPower(1);
            armPosition = pivotMotor.getCurrentPosition();
        } else if (gamepad1.left_bumper) {
            piviotPID.setPID(0.00005, 0, 0.000000000001);
            pivotMotor.setPower(-1);
            armPosition = pivotMotor.getCurrentPosition();
        }
        System.out.println("Current Position" + armPosition);
        // PID control logic for pivoting
        if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            piviotPID.setSetPoint(armPosition);
            piviotPID.setMaxOutput(1);
            piviotPID.setMinOutput(-1);
            piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());

            // Set motor power using the PID result
            pivotMotor.setPower(-piviotPID.getResult());
        }
    }


    // Get the current position of the motor in encoder ticks
    public static double getCurrentPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public static double getCurrentPositionWithLimitSwitch() {

        if (!limitSwitch.getState()) {
            pos = ArmSubsystem.getCurrentPosition();



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

    public static void gamepad1Climb(Gamepad gamepad1) {
        if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            piviotPID.setSetPoint(armPosition);
            piviotPID.setMaxOutput(1);
            piviotPID.setMinOutput(-1);
            piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());

            // Set motor power using the PID result
            pivotMotor.setPower(-piviotPID.getResult());
        }
    }

    public static void controlPiviotwithOperator(Gamepad gamepad2, PID piviotPID) {

        if (gamepad2.square) {
            piviotPID.setPID(0.015, 0, 0.01);
            armPosition = -3000;

        } else if (gamepad2.b) {
            piviotPID.setPID(0.015, 0, 0.01);
            armPosition = 0;

        } else if (gamepad2.dpad_down) {
            piviotPID.setPID(0.015, 0, 0.01);
            armPosition = -3365;
        }
        if (gamepad2.dpad_down) {
            Wrist.wrist.setPosition(0.49);
        }
            if (gamepad2.triangle) {
                Wrist.wrist.setPosition(0.8);
            }
            if (gamepad2.triangle) {
                piviotPID.setPID(0.015, 0, 0.01);
                armPosition = -1936;
            }

            if (gamepad2.right_bumper) {
                piviotPID.setPID(0.00005, 0, 0.0000000001);
                pivotMotor.setPower(1);
                armPosition = pivotMotor.getCurrentPosition();
            } else if (gamepad2.left_bumper) {
                piviotPID.setPID(0.00005, 0, 0.000000000001);
                pivotMotor.setPower(-1);
                armPosition = pivotMotor.getCurrentPosition();
            }
            System.out.println("Current Position" + armPosition);
            // PID control logic for pivoting
            if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                piviotPID.setSetPoint(armPosition);
                piviotPID.setMaxOutput(1);
                piviotPID.setMinOutput(-1);
                piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());

                // Set motor power using the PID result
                pivotMotor.setPower(-piviotPID.getResult());
            }
        }
    }
