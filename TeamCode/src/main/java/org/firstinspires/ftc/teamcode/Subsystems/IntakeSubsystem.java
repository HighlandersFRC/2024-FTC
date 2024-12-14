
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


public class IntakeSubsystem extends Subsystem {
    public static CRServo intake;
    public static NormalizedColorSensor colorSensor;
    public IntakeSubsystem(String name) {
        super();
    }

    public static void initialize(HardwareMap hardwareMap) {
        intake = hardwareMap.crservo.get("intake");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public static void setPower(double power) {
        intake.setPower(power);
    }

    public static void contolIntake(Gamepad gamepad1) {
           if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad1.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
    }
        }




