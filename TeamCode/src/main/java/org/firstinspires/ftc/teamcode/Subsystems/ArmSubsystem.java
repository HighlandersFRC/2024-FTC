package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmSubsystem extends Subsystem {
    public static DcMotor pivotMotor;
    public static DigitalChannel limitSwitch; // Limit switch for arm position
    private static double armPosition = Double.NaN; // Target arm position
    static PID piviotPID = new PID(0.15, 0, 1);
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

    // Set motor power
    public static void setPower(double power) {
        pivotMotor.setPower(power);
    }

    public static void controlPivot(Gamepad gamepad1, PID pivotPID) {
        // Adjust the arm position based on the limit switch state
        if (limitSwitch.getState()) { // Limit switch is pressed
            armPosition = 1736; // Set to a specific target position
        } else { // Limit switch is not pressed
            armPosition = 0; // Default or safe position
        }

        // Update the PID controller with the target position
        pivotPID.setSetPoint(armPosition);
        pivotPID.setMaxOutput(1);
        pivotPID.setMinOutput(-1);
        pivotPID.updatePID(pivotMotor.getCurrentPosition());

        // Set motor power using the PID result
        pivotMotor.setPower(-pivotPID.getResult());

        // Allow manual override for fine control
        if (gamepad1.dpad_up) {
             armPosition=1736;
        } else if (gamepad1.dpad_down) {
            armPosition=0; // Move arm downward directly
        }
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
        if (gamepad1.dpad_up) {
            armPosition=1736;
        } else if (gamepad1.dpad_down) {
            armPosition=0; // Move arm downward directly
        }
    }


    // Get the current position of the motor in encoder ticks
    public static int getCurrentPosition() {
        return pivotMotor.getCurrentPosition();
    }

    // Get the current target arm position
    public static double getTargetPosition() {
        return armPosition;
    }
}

