package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.IntakeDefault;
public class IntakeSubsystem extends Subsystem {
    private Servo RightIntake;
    private Servo LeftIntake;

    private double lastRightIntakePosition = -1;
    private double lastLeftIntakePosition = -1;
    public IntakeSubsystem(String name) {
        super(name);
    }
    public double getPositionRight() {
        return RightIntake.getPosition();
    }

    public double getPositionLeft() {
        return LeftIntake.getPosition();
    }

    public void initialize(HardwareMap hardwareMap) {
        RightIntake = hardwareMap.servo.get("IntakeRight");
        LeftIntake = hardwareMap.servo.get("IntakeLeft");
    }




    public void controlIntake(Gamepad gamepad1) {
        double rightIntakePosition = lastRightIntakePosition; // Default to the last position
        double leftIntakePosition = lastLeftIntakePosition;

        if (gamepad1.right_trigger > 0) {
            // Fully open the intake
            rightIntakePosition = 1.0;
            leftIntakePosition = 0.0;
        } else if (gamepad1.left_trigger > 0) {
            // Clamp the intake
            rightIntakePosition = 0.5;
            leftIntakePosition = 0.5;
        }

        // Update servo positions only if they are different from the last known positions
        if (rightIntakePosition != lastRightIntakePosition || leftIntakePosition != lastLeftIntakePosition) {
            RightIntake.setPosition(rightIntakePosition);
            LeftIntake.setPosition(leftIntakePosition);

            // Update last known positions
            lastRightIntakePosition = rightIntakePosition;
            lastLeftIntakePosition = leftIntakePosition;

            // Debugging
            System.out.println("Servo positions updated: Right=" + rightIntakePosition + ", Left=" + leftIntakePosition);
        }
    }




    public void setPosition(double RightPos, double LeftPos) {
        RightIntake.setPosition(RightPos);
        RightIntake.setPosition(LeftPos);
    }


    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(new IntakeDefault());
    }

    @Override
    public Command getDefaultCommand() {
        return new IntakeDefault();
    }
}
