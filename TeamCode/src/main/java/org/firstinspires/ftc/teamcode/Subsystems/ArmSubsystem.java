package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ArmDefault;

public class ArmSubsystem extends Subsystem {
    private DcMotor pivotMotor;
    private DigitalChannel limitSwitch;
    private double pos = 0;

    public ArmSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        initialize(hardwareMap);
    }

    private void initialize(HardwareMap hardwareMap) {
        try {
            pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
            limitSwitch = hardwareMap.digitalChannel.get("limitSwitch");

            // Ensure limitSwitch is set to input mode
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            throw new IllegalStateException("Failed to initialize ArmSubsystem: " + e.getMessage());
        }
    }

    public void setPower(double power) {
        if (pivotMotor != null) {
            pivotMotor.setPower(power);
        }
    }

    public double getCurrentPosition() {
        return pivotMotor != null ? pivotMotor.getCurrentPosition() : 0;
    }

    public double getCurrentPositionWithLimitSwitch() {
        if (limitSwitch != null && !limitSwitch.getState()) {
            pos = getCurrentPosition();
        }
        return getCurrentPosition() - pos;
    }

    public void ArmMovement(Gamepad gamepad1) {
        if (gamepad1.left_bumper) {
            setPower(1);
        } else if (gamepad1.right_bumper) {
            setPower(-1);
        } else {
            setPower(0);
        }
    }

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command);
    }

    @Override
    public Command getDefaultCommand() {
        return new ArmDefault(this); // Pass the current instance of ArmSubsystem
    }
}
