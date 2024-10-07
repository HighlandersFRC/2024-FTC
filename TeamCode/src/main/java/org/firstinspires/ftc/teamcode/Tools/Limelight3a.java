package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

@TeleOp
public class Limelight3a extends LinearOpMode {
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {

        Peripherals.initialize(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0);
        limelight.getStatus();
        limelight.start();


        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();


            if (result != null) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }


            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }
            }


            double robotYaw = Peripherals.getYaw();
            limelight.updateRobotOrientation(robotYaw);
            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");

                }
                telemetry.update();
            }



            }

    limelight.stop();
        }

        }



