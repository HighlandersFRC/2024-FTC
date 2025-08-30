
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class NewRobot {

    // Instance variables for subsystems
    public Drive drive;
    public NewIntakeSubsystem intake;
    public NewWristSubsystem wrist;
    public NewArmSubsystem arm;
    public NewElevatorSubsystem elevator;
    public Superstructure superstructure;

    public NewRobot(HardwareMap hardwareMap) {
        this.drive = new Drive("drive", hardwareMap);
        this.intake = new NewIntakeSubsystem("intakeSubsystem");
        this.wrist = new NewWristSubsystem("wrist");
        this.arm = new NewArmSubsystem("arm");
        this.elevator = new NewElevatorSubsystem("elevator");
        this.superstructure = new Superstructure("superstructure");
    }

    public void run() {

    }

    // Initialize hardware for all subsystems
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = new Drive("drive", hardwareMap);
        this.intake = new NewIntakeSubsystem("intakeSubsystem");
        this.wrist = new NewWristSubsystem("wrist");
        this.arm = new NewArmSubsystem("arm");
        this.elevator = new NewElevatorSubsystem("elevator");
        this.superstructure = new Superstructure("superstructure");
    }

}
