
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

    // Add this method to handle gamepad inputs for intake control
    public static void controlIntakeRedAlliance(Gamepad gamepad1) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        double red = color.red;
        double blue = color.blue;
        double green = color.green;

        if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(1);
        } else if (gamepad1.right_trigger != 0) {
            IntakeSubsystem.setPower(-1);
        } else {
            System.out.println("ran");
            IntakeSubsystem.setPower(0);
        }

         if (blue > red && blue > green && blue > 0.01) {
            System.out.println("Blue");
            IntakeSubsystem.intake.setPower(-1);
        } else {
            System.out.println("None");
            IntakeSubsystem.intake.setPower(0);
        }

    }

    public static void contolIntakeBlueAlliance(Gamepad gamepad1) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        double red = color.red;
        double blue = color.blue;
        double green = color.green;

        if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad1.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
        if (gamepad1.right_trigger != 1 && gamepad1.left_trigger != 1) {
            if (red > blue && red > green && red > 0.01) {
                gamepad1.rumble(100);
                System.out.println("Red");
                IntakeSubsystem.intake.setPower(-1);
            }
        }
    }

    public static void contolIntakeWithDriveOperatorRedAlliance(Gamepad gamepad2, Gamepad gamepad1) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        double red = color.red;
        double blue = color.blue;
        double green = color.green;

        if (gamepad2.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad2.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
        if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad1.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
        if (gamepad1.right_trigger != 1 && gamepad1.left_trigger != 1) {
            if (blue > red && blue > green && blue > 0.01) {
                gamepad1.rumble(100);
                System.out.println("Blue");
                IntakeSubsystem.intake.setPower(1);
            }
        }

    }

    public static void contolIntakeWithDriverOperatorBlueAlliance(Gamepad gamepad2, Gamepad gamepad1) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        double red = color.red;
        double blue = color.blue;
        double green = color.green;

        if (gamepad2.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad2.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
        if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad1.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
        if (gamepad1.right_trigger != 1 && gamepad1.left_trigger != 1) {
            if (red > blue && red > green && red > 0.01) {
                gamepad1.rumble(100);
                System.out.println("Red");
                IntakeSubsystem.intake.setPower(1);
            }
        }
    }
}

