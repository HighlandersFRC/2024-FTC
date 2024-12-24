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
import org.firstinspires.ftc.teamcode.Tools.Robot;


@TeleOp
public class CommandKitBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Robot.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();

        ArmSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        Mouse.init(hardwareMap);

        ArmCommand Score = new ArmCommand(Robot.arm,DegreesToEncoderTicks(120));
        ArmCommand Zero= new ArmCommand(Robot.arm,DegreesToEncoderTicks(0));
        ArmCommand pickUp = new ArmCommand(Robot.arm,DegreesToEncoderTicks(215));
        ArmCommand Enter= new ArmCommand(Robot.arm,DegreesToEncoderTicks(150));

        scheduler = new CommandScheduler();

        Intake intakeCommand = new Intake(Robot.intake);
        Outtake outtakeCommand = new Outtake(Robot.intake);

        WristCommands leftWrist = new WristCommands(Robot.wrist,0.4);
        WristCommands rightWrist = new WristCommands(Robot.wrist,0.8);
        WristCommands zeroWrist = new WristCommands(Robot.wrist,0);


        while(opModeIsActive()) {
            if (gamepad1.right_trigger != 0) {
                StopTheIntake = false;
                scheduler.schedule(intakeCommand);
            } else if (gamepad1.left_trigger != 0) {
                StopTheIntake = false;
                scheduler.schedule(outtakeCommand);
            } else {
                StopTheIntake = true;
            }

            if (gamepad1.y) {
                scheduler.schedule(Score);
            } else if (gamepad1.b) {
                scheduler.schedule(Zero);
            }
              else if(gamepad1.x){
                  scheduler.schedule(Enter);
            }
            else if(gamepad1.a){
                scheduler.schedule(pickUp);
            }


            Drive.FeildCentric(gamepad1);
            scheduler.printCurrentCommands();
            telemetry.addData("commands", CommandScheduler.getInstance().printCurrentCommandsTele());
            scheduler.run();

            double tolerance = 100;
            double currentPosition = ArmSubsystem.getCurrentPositionWithLimitSwitch();
            telemetry.addData("a",(Math.abs(currentPosition + 1900)) <= tolerance);
            telemetry.addData("b",Math.abs(currentPosition + 1900));

            telemetry.addData("en",DegreesToEncoderTicks(120));
            telemetry.addData("Mouse Sensor Y:", Mouse.getX());
            telemetry.addData("Mouse Sensor X:", Mouse.getY());
            telemetry.addData("Piviot Arm Posiotion:", ArmSubsystem.getCurrentPositionWithLimitSwitch());
            telemetry.update();
        }




    }
}
