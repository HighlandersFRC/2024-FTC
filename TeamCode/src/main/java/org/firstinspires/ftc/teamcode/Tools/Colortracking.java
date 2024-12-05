package org.firstinspires.ftc.teamcode.Tools;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

import java.util.List;

@TeleOp
public class Colortracking extends LinearOpMode {
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {

        Peripherals.initialize(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(1);
        limelight.getStatus();
        limelight.start();


        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
            for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                double x = colorTarget.getTargetXDegrees(); // Where it is (left-right)
                double y = colorTarget.getTargetYDegrees();
                Pose3D Pose3d = colorTarget.getRobotPoseFieldSpace();// Where it is (up-down)
                double area = colorTarget.getTargetArea(); // size (0-100)

                telemetry.addData("Color Target", "takes up " + area + "% of the image");
            }
                telemetry.update();
            }


        return null;
    }


    }




