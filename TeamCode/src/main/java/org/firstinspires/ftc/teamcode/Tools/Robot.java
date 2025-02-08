package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class Robot {
    public static String CURRENT_STATE = "Auto";
    public static double CURRENT_PIVOT;
    public static String PIVOT_STATE = "PID";
    public static double PIVOT_RAW_POWER = 0;
    public static double CURRENT_WRIST;
    public static double CURRENT_INTAKE_POWER;
    public static double CURRENT_ELEVATOR = 0;
    public static double elevatorPower = 0;

    public static void initialize(HardwareMap hardwareMap) {
        Peripherals.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        FieldOfMerit.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Pivot.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Elevators.initialize(hardwareMap);
    }

    public static Elevators elevators = new Elevators("elevators");
    public static Intake intake = new Intake("intake");
    public static Wrist wrist = new Wrist("wrist");
    public static Pivot pivot = new Pivot("pivot");

    public static void run() {
        pivot.checkForZero();

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
