
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public static double CURRENT_PIVOT;
    public static double CURRENT_WRIST;
    public static double CURRENT_INTAKE_POWER;
    public static double CURRENT_ELEVATOR;
    public static double elevatorPower = 0;

    // Instance variables for subsystems
    public Drive drive;
    public IntakeSubsystem intake;
    public Wrist wrist;
    public ArmSubsystem arm;

    // Constructor to initialize subsystems with hardwareMap and telemetry
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize subsystems with appropriate hardware
        this.drive = new Drive("drive", hardwareMap, telemetry);
        this.intake = new IntakeSubsystem("intakeSubsystem");
        this.wrist = new Wrist("wrist");
        this.arm = new ArmSubsystem("arm", hardwareMap);
    }

    // Initialize hardware for all subsystems
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = new Drive("drive", hardwareMap, telemetry);
        this.intake = new IntakeSubsystem("intakeSubsystem");
        this.wrist = new Wrist("wrist");
        this.arm = new ArmSubsystem("arm", hardwareMap);
    }

    // Elevator power calculation based on button states
    public static double elevatorPowerCalc(boolean right, boolean left) {
        if (right) {
            return 1;
        } else if (left) {
            return -1;
        } else {
            return 0;
        }
    }
}
