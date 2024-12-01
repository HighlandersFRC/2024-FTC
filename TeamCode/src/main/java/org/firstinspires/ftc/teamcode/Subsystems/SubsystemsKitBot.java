package org.firstinspires.ftc.teamcode.Subsystems;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.Mouse;

public class SubsystemsKitBot extends Subsystem {

    public static void init(HardwareMap hardwareMap) {
        ArmSubsystem.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        Mouse.init(hardwareMap);
    }

    public static void control() {
        ArmSubsystem.controlPivot(gamepad2,piviotPID);
        IntakeSubsystem.controlIntake(gamepad2);
        Wrist.controlWrist(gamepad2);
        Drive.FeildCentric(gamepad1);
        ArmSubsystem.gamepad1Climb(gamepad1, piviotPID);
        Mouse.update();
    }

}