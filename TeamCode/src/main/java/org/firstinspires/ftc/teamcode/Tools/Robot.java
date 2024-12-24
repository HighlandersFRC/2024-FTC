package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class Robot {
    public static double CURRENT_PIVOT;
    public static double CURRENT_WRIST;
    public static double CURRENT_INTAKE_POWER;
    public static double CURRENT_ELEVATOR;
    public static double elevatorPower = 0;

    public static void initialize(HardwareMap hardwareMap) {

        Drive.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        ArmSubsystem.initialize(hardwareMap);
    }

    public static Drive drive = new Drive("drive");
    public static IntakeSubsystem intake = new IntakeSubsystem("intakeSubsystem");
    public static Wrist wrist = new Wrist("wrist");
    public static ArmSubsystem arm = new ArmSubsystem("arm");

    public static void run() {

    }

    public static double elevatorPowerCalc(boolean right, boolean left) {

        if (right) {
            return 1;
        } else if (left) {
            return -1;
        } else if (right && left) {
            return 0;
        } else if (!right && !left) {
            return 0;
        }
        return 0;
    }
}
