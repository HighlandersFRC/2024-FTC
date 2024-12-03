package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
@TeleOp
public class ColorSensor extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        waitForStart();
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        IntakeSubsystem.initialize(hardwareMap);
        while(opModeIsActive()) {
            NormalizedRGBA color = colorSensor.getNormalizedColors();
            double red = color.red;
            double blue = color.blue;
            double green = color.green;
//            if(blue > red && blue > green) {
//                IntakeSubsystem.intake.setPower(1);
//            } else {
//                IntakeSubsystem.intake.setPower(0);
//            }
            telemetry.addData("Current Color", colorSensor.getNormalizedColors());
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();
        }
    }
}
