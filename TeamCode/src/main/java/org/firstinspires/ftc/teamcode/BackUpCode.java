//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Drive;
//import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
//@TeleOp
//public class BackUpCode extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//       DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
//       IntakeSubsystem intakeSubsystem = new IntakeSubsystem("Intake", hardwareMap);
//       Wrist wristSubsystem = new Wrist("Wrist", hardwareMap);
//       Drive driveSubsystem = new Drive("Drive", hardwareMap);
//       ElevatorSubsystem elevator = new ElevatorSubsystem("Elevator", hardwareMap);
//       pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       waitForStart();
//        while (opModeIsActive()) {
//
//            driveSubsystem.FieldCentric(gamepad1);
//
//            elevator.manual(gamepad2);
//            intakeSubsystem.controlIntake(gamepad2);
//            wristSubsystem.contolWrist(gamepad2);
//            if (gamepad2.left_bumper) {
//                pivotMotor.setPower(0.5);
//            } else if (gamepad2.right_bumper) {
//                pivotMotor.setPower(-0.5);
//            } else {
//                pivotMotor.setPower(0);
//            }
//        }
//    }
//}
