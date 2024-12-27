package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Commands.StopIntake.StopTheIntake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

@TeleOp
public class CommandKitBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and systems

        CommandScheduler scheduler = new CommandScheduler();

        ArmSubsystem armSubsystem = new ArmSubsystem("arm",hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem("intakeSubsystem");
        IntakeSubsystem.initialize(hardwareMap);
        Wrist wrist = new Wrist("wrist");
        wrist.initialize(hardwareMap);
        Drive drive = new Drive("drive",hardwareMap,telemetry);
        drive.initialize(hardwareMap);

        // Subsystem initialization with try-catch blocks for safety

        try {
            drive = new Drive("drive", hardwareMap, telemetry);
            drive.initialize(hardwareMap);

            armSubsystem = new ArmSubsystem("arm", hardwareMap);

            wrist = new Wrist("wrist");
            wrist.initialize(hardwareMap);

            IntakeSubsystem.initialize(hardwareMap);
            Mouse.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Initialization Error", e.getMessage());
            telemetry.update();
            return; // Exit if initialization fails
        }

        ArmCommand Score = new ArmCommand(armSubsystem, DegreesToEncoderTicks(120));
        ArmCommand Zero = new ArmCommand(armSubsystem, DegreesToEncoderTicks(0));
        ArmCommand pickUp = new ArmCommand(armSubsystem, DegreesToEncoderTicks(215));
        ArmCommand Enter = new ArmCommand(armSubsystem, DegreesToEncoderTicks(150));

        // Intake and wrist commands
        Intake intakeCommand = new Intake(intake);
        Outtake outtakeCommand = new Outtake(intake);

        WristCommands leftWrist = new WristCommands(wrist, 0.4);
        WristCommands rightWrist = new WristCommands(wrist, 0.8);
        WristCommands zeroWrist = new WristCommands(wrist, 0);

        waitForStart();

        // Main loop
        int loopCount = 0; // Loop counter for throttling telemetry
        while (opModeIsActive()) {
            try {
                // Drive logic
                drive.FeildCentric(gamepad1);

                // Command scheduling
                if (gamepad1.right_trigger > 0 && !scheduler.isCommandScheduled(intakeCommand)) {
                    StopTheIntake = false;
                    scheduler.schedule(intakeCommand);
                } else if (gamepad1.left_trigger > 0 && !scheduler.isCommandScheduled(outtakeCommand)) {
                    StopTheIntake = false;
                    scheduler.schedule(outtakeCommand);
                } else {
                    StopTheIntake = true;
                }

                if (gamepad1.y && !scheduler.isCommandScheduled(Score)) {
                    scheduler.schedule(Score);
                } else if (gamepad1.b && !scheduler.isCommandScheduled(Zero)) {
                    scheduler.schedule(Zero);
                } else if (gamepad1.x && !scheduler.isCommandScheduled(Enter)) {
                    scheduler.schedule(Enter);
                } else if (gamepad1.a && !scheduler.isCommandScheduled(pickUp)) {
                    scheduler.schedule(pickUp);
                }

                // Run scheduled commands
                scheduler.run();

                // Telemetry updates
                if (loopCount % 10 == 0) { // Update telemetry every 10 loops
                    telemetry.addData("Mouse Sensor", "X: %f, Y: %f", Mouse.getX(), Mouse.getY());
                    telemetry.addData("Pivot Arm Position", armSubsystem.getCurrentPositionWithLimitSwitch());
                    telemetry.update();
                }
                loopCount++;
            } catch (Exception e) {
                telemetry.addData("Runtime Error", e.getMessage());
                telemetry.update();
            }
        }
    }
}
