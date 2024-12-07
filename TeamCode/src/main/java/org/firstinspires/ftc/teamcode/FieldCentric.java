package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Robot;

@TeleOp
public class FieldCentric extends LinearOpMode {

    private final PID pivotPID = new PID(0.1, 0.004, 0.095);
    private final PID elevatorPID = new PID(0.01, 0.0, 0.008);

    private static final double PIVOT_LOW_POSITION = 100;
    private static final double PIVOT_HIGH_POSITION = 700;

    private static final double ELEVATOR_INCREMENT = 200;
    private static final double ELEVATOR_SUBMERSIBLE_DISTANCE = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Pivot.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Elevators.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);

        elevatorPID.setSetPoint(0);

        pivotPID.setMaxOutput(0.5);
        pivotPID.setMinInput(180);
        pivotPID.setMaxInput(-180);

        elevatorPID.setMaxOutput(0.5);
        waitForStart();
        pivotPID.setSetPoint(-14);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            Mouse.update();
            FinalPose.poseUpdate();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.start) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            double pivotPower = pivotPID.updatePID(Pivot.getAngle());

            if (gamepad1.b) {
                pivotPID.setSetPoint(45);
            } else if (gamepad1.y) {
                pivotPID.setSetPoint(88);
            }else if (gamepad1.a){
                pivotPID.setSetPoint(-10);
            }
            if (Pivot.getAngle()< 15 && pivotPID.getSetPoint() < 0){
                Pivot.setPower(0);
                Pivot.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }else {
                Pivot.setPower(pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Math.toRadians(Pivot.getAngle()) + Constants.ARM_BALANCE_OFFSET)));

            }



            if (gamepad1.left_bumper) {
                elevatorPID.setSetPoint(Elevators.getLeftEncoder() - ELEVATOR_INCREMENT);
            }
            else if (gamepad1.right_bumper) {
                elevatorPID.setSetPoint(Elevators.getLeftEncoder() + ELEVATOR_INCREMENT);
            }

            Elevators.moveLeftElevator(elevatorPID.updatePID((Elevators.getLeftEncoder() + Elevators.getRightEncoder()) / 2));
            Elevators.moveRightElevator(elevatorPID.updatePID((Elevators.getLeftEncoder() + Elevators.getRightEncoder()) / 2));

           double intakePower = gamepad1.left_trigger - gamepad1.right_trigger;
           if (!(intakePower == 1) || !(intakePower == -1)){
               Intake.move(intakePower);
           }else {
               Intake.stopIntake();
           }

            if (gamepad1.dpad_down){
                Wrist.move(1);
            }
            if (gamepad1.dpad_up){
                Wrist.move(0.1);
            }

            if (gamepad1.x) {
                pivotPID.setSetPoint(90);
/*
                waitFor(0.5);
*/
                Wrist.move(0.1);
                elevatorPID.setSetPoint(1000);
            }
            if (gamepad1.dpad_right){
                elevatorPID.setSetPoint(0);
                waitFor(1);
                pivotPID.setSetPoint(-5);
/*
                waitFor(0.5);
*/
                Wrist.move(1);
            }

/*
            Pivot.checkForZero();
*/

            double botHeading = 0;
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            Drive.drive(frontLeftPower, frontRightPower, -backLeftPower, backRightPower);

            // Telemetry
            telemetry.addData("Yaw", botHeading);

            telemetry.addLine("Pivot")
                    .addData("Encoder", Pivot.getEncoderPosition())
                    .addData("Set Point", pivotPID.getSetPoint())
                    .addData("Pivot Angle", Pivot.getAngle())
                    .addData("Power", (pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Pivot.getAngle() + 22))))
                    .addData("PID Output", pivotPID.getResult());

            telemetry.addLine("Elevator")
                    .addData("Left Encoder", Elevators.getLeftEncoder())
                    .addData("Right Encoder", Elevators.getRightEncoder())
                    .addData("Set Point", elevatorPID.getSetPoint());
            telemetry.addLine("Pose")
                    .addData("x", FinalPose.x)
                    .addData("y", FinalPose.y)
                    .addData("current", FieldOfMerit.currentState)
                    .addData("yaw", FinalPose.yaw);
            telemetry.update();
        }
    }
    public void waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Waiting", "%.1f seconds left", seconds - timer.seconds());
            telemetry.update();
        }
    }
}