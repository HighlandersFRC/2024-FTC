
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
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
    public IntakeSubsystem intakeSubsystem;
    public Wrist wrist;
    public ArmSubsystem arm;
    public ElevatorSubsystem elevator;

    public Robot(HardwareMap hardwareMap) {
        this.drive = new Drive("drive", hardwareMap);
        this.intakeSubsystem = new IntakeSubsystem("intakeSubsystem", hardwareMap);
        this.wrist = new Wrist("wrist", hardwareMap);
        this.arm = new ArmSubsystem("arm", hardwareMap);
        this.elevator = new ElevatorSubsystem("elevator", hardwareMap);
    }

    public void run() {

    }

    // Initialize hardware for all subsystems
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = new Drive("drive", hardwareMap);
        this.intakeSubsystem = new IntakeSubsystem("intakeSubsystem", hardwareMap);
        this.wrist = new Wrist("wrist", hardwareMap);
        this.arm = new ArmSubsystem("arm", hardwareMap);
        this.elevator = new ElevatorSubsystem("elevator", hardwareMap);
    }

}
