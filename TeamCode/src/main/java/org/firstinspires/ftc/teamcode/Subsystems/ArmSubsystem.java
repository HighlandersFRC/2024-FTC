package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.encodersToDeg;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends Subsystem {
    public static DcMotor pivotMotor;
    public static DigitalChannel limitSwitch;
    private static double armPosition = encodersToDeg(0);
    protected static double pos = 0;
    private static boolean manualControlActive = false; // Tracks if manual control is being used

    public static void initialize(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        limitSwitch = hardwareMap.digitalChannel.get("limitSwitch");
    }

    public static void setPower(double power) {
        pivotMotor.setPower(power);
    }

    public static double getCurrentPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public static double getCurrentPositionWithLimitSwitch() {
        if (!limitSwitch.getState()) {
            pos = ArmSubsystem.getCurrentPosition();
        }
        return ArmSubsystem.getCurrentPosition() - pos;
    }

    public static void ArmMovement(Gamepad gamepad1) {
        manualControlActive = false; // Reset the manual control flag

        if (gamepad1.left_bumper) {
            ArmSubsystem.setPower(1);
            manualControlActive = true; // Manual control is active
        } else if (gamepad1.right_bumper) {
            ArmSubsystem.setPower(-1);
            manualControlActive = true; // Manual control is active
        } else {
            ArmSubsystem.setPower(0); // Stop movement
        }
    }

    public static boolean isManualControlActive() {
        return manualControlActive;
    }
}
