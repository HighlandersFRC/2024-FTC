package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ArmDefault;
import org.firstinspires.ftc.teamcode.Commands.ElevatorDown;
import org.firstinspires.ftc.teamcode.Commands.ElevatorUp;
import org.firstinspires.ftc.teamcode.Commands.StopElevator;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.json.JSONException;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;

@TeleOp
public class CommandKitBot extends LinearOpMode {
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        dashboard = FtcDashboard.getInstance();
        ArmSubsystem armSubsystem = new ArmSubsystem("arm", hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem("intakeSubsystem", hardwareMap);
        Wrist wrist = new Wrist("wrist", hardwareMap);
        Drive driveSubsystem = new Drive("Drive", hardwareMap);

        Robot robot = new Robot(hardwareMap);
        robot.arm = armSubsystem;
        robot.drive = driveSubsystem;
        robot.intakeSubsystem = intake;
        robot.wrist = wrist;
        scheduler.setRobot(robot);

        double Pos = 0;

        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousDpadDown = false;

        ArmDefault armDefault = new ArmDefault(armSubsystem);
        waitForStart();

        while (opModeIsActive()) {




            // Button Press Detection
            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentDpadDown = gamepad1.dpad_down;

            if (currentB && !previousB) {
                Pos = DegreesToEncoderTicks(0);

                ArmCommand pos = new ArmCommand(robot.arm, Pos);
                scheduler.schedule(pos);
            }else if (currentY && !previousY) {
                Pos = DegreesToEncoderTicks(60);

                ArmCommand pos = new ArmCommand(robot.arm, Pos);
                scheduler.schedule(pos);
            } else if (currentDpadDown && !previousDpadDown) {
                Pos = DegreesToEncoderTicks(45);

                ArmCommand pos = new ArmCommand(robot.arm, Pos);
                scheduler.schedule(pos);
            } else if (currentX && !previousX) {
                Pos = DegreesToEncoderTicks(90);

                ArmCommand pos = new ArmCommand(robot.arm, Pos);
                scheduler.schedule(pos);
            }

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousDpadDown = currentDpadDown;




                scheduler.run();


            double currentPosition = armSubsystem.getCurrentPositionWithLimitSwitch();
            scheduler.printCurrentCommands();

            telemetry.addData("Target Positions", "Score: %f, Enter: %f, Pick Up: %f, Zero: %f",
                    DegreesToEncoderTicks(70), DegreesToEncoderTicks(100), DegreesToEncoderTicks(120), DegreesToEncoderTicks(0));
            telemetry.addData("Is Finished", Math.abs(currentPosition - Pos) <= 7);
            telemetry.addData("Arm Power", armSubsystem.getPower());
            telemetry.addData("Current Target", Pos);
            telemetry.update();


            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Arm Degrees", getDegrees(armSubsystem.getCurrentPositionWithLimitSwitch()));
            packet.put("Drive Degrees", getDegrees(driveSubsystem.leftBackPos()));
            packet.put("Mouse Sensor X", -Mouse.getY());
            packet.put("Mouse Sensor Y", -Mouse.getX());
            packet.put("Mouse Sensor theta", -Mouse.getTheta());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
