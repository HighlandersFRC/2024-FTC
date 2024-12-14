
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class IntakeSubsystem extends Subsystem {
    public static CRServo intake;

    public IntakeSubsystem(String name) {
        super();
    }

    public static void initialize(HardwareMap hardwareMap) {
        intake = hardwareMap.crservo.get("intake");
    }

    public static void setPower(double power) {
        intake.setPower(power);
    }

    public static double getPower(){

        return intake.getPower();
    }

    public static void contolIntake(Gamepad gamepad1) {
           if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(1);
        } else if (gamepad1.right_trigger != 0) {
               System.out.println("it ran");
            IntakeSubsystem.setPower(-1);
        } else {
            IntakeSubsystem.setPower(0);
        }

    }
        }




