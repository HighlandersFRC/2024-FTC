package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
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
        IntakeSubsystem intake = new IntakeSubsystem("intakeSubsystem", hardwareMap);
        Wrist wrist = new Wrist("wrist", hardwareMap);
        Drive driveSubsystem = new Drive("Drive", hardwareMap);
        ElevatorSubsystem elevator = new ElevatorSubsystem("Elevator", hardwareMap);

        // Subsystem initialization with try-catch blocks for safety

        ArmCommand Score = new ArmCommand(armSubsystem, DegreesToEncoderTicks(70));
        ArmCommand Zero = new ArmCommand(armSubsystem, DegreesToEncoderTicks(0));
        ArmCommand pickUp = new ArmCommand(armSubsystem, DegreesToEncoderTicks(120));
        ArmCommand Enter = new ArmCommand(armSubsystem, DegreesToEncoderTicks(100));

        ElevatorCommand ScoreEle = new ElevatorCommand(elevator, -5000);
        ElevatorCommand Center = new ElevatorCommand(elevator, -1000);
        ElevatorCommand ZeroEle = new ElevatorCommand(elevator, -200);
//ArmCommand UP = new ArmCommand(armSubsystem, armSubsystem.getCurrentPositionWithLimitSwitch());
//ArmCommand STOP = new ArmCommand(armSubsystem, armSubsystem.getCurrentPositionWithLimitSwitch());
//        // Intake and wrist commands
        Intake intakeCommand = new Intake(intake);
        Outtake outtakeCommand = new Outtake(intake, 1);

        WristCommands leftWrist = new WristCommands(wrist, 0.4);
        WristCommands rightWrist = new WristCommands(wrist, 0.8);
        WristCommands zeroWrist = new WristCommands(wrist, 0);

        waitForStart();

        // Main loop
        int loopCount = 0; // Loop counter for throttling telemetry
        while (opModeIsActive()) {

                // Drive logic
                driveSubsystem.FeildCentric(gamepad1);

                // Command scheduling
                if (gamepad1.right_trigger > 0 && !scheduler.isCommandScheduled(intakeCommand)) {
                    scheduler.schedule(intakeCommand);
                } else if (gamepad1.left_trigger > 0 && !scheduler.isCommandScheduled(outtakeCommand)) {
                    scheduler.schedule(outtakeCommand);
                }

                if (gamepad1.b && !scheduler.isCommandScheduled(Zero)) {
                    scheduler.schedule(Zero);
                    scheduler.schedule(ZeroEle);
                 } else if (gamepad1.x && !scheduler.isCommandScheduled(Enter)) {
                    scheduler.schedule(Enter);
                    scheduler.schedule(Center);
                } else if (gamepad1.y &&!scheduler.isCommandScheduled(Score)) {
                    scheduler.schedule(Score);
                    scheduler.schedule(ScoreEle);
                } else if (gamepad1.dpad_down && !scheduler.isCommandScheduled(pickUp)) {
                    scheduler.schedule(pickUp);
                    scheduler.schedule(ZeroEle);
                }


//            if (gamepad1.right_bumper){
//                scheduler.schedule(UP);
//            } else {
//                scheduler.schedule(STOP);
//            }

                if (gamepad1.dpad_up){
                    scheduler.schedule(zeroWrist);
                }
                else if (gamepad1.dpad_right){
                    scheduler.schedule(rightWrist);
                }
                else if (gamepad1.dpad_left){
                    scheduler.schedule(leftWrist);
                }

                // Run scheduled commands
                scheduler.run();
            System.out.println("Pivot Arm Positon " + armSubsystem.getCurrentPositionWithLimitSwitch());
            System.out.println("Score " + DegreesToEncoderTicks(70));
            System.out.println("Enter " + DegreesToEncoderTicks(100));
            System.out.println("Pick Up " + DegreesToEncoderTicks(120));
            System.out.println("Zero "+ DegreesToEncoderTicks(0));
                // Telemetry updates
                 // Update telemetry every 10 loops
                    telemetry.addData("Mouse Sensor", "X: %f, Y: %f", Mouse.getX(), Mouse.getY());
                    telemetry.addData("Pivot Arm Position", armSubsystem.getCurrentPositionWithLimitSwitch());
            telemetry.addData("Score " , DegreesToEncoderTicks(70));
           telemetry.addData("Enter " , DegreesToEncoderTicks(100));
            telemetry.addData("Pick Up " ,DegreesToEncoderTicks(120));
         telemetry.addData("Zero ", DegreesToEncoderTicks(0));

                    telemetry.update();



    }
}
}