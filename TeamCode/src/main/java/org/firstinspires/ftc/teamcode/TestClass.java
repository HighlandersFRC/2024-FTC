package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;


@TeleOp
public class TestClass extends LinearOpMode {
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        dashboard = FtcDashboard.getInstance();
        ArmSubsystem armSubsystem = new ArmSubsystem("arm", hardwareMap);
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem("elevator", hardwareMap);
        Drive drive = new Drive("drive", hardwareMap);
        while (opModeIsActive()) {
//         armSubsystem.ArmMovement(gamepad1);
         elevatorSubsystem.contolElevatorSetPoint(gamepad1);
         drive.FeildCentric(gamepad1);
            TelemetryPacket packet = new TelemetryPacket();
                packet.put("Degrees-Arm", getDegrees(armSubsystem.getCurrentPositionWithLimitSwitch()));
                packet.put("Degrees-Elevator", elevatorSubsystem.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
