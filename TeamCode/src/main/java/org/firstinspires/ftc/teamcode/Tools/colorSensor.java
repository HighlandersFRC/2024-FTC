package org.firstinspires.ftc.teamcode.Tools;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class colorSensor extends LinearOpMode {


  NormalizedColorSensor colorSensor;

  View relativeLayout;

  @Override
  public void runOpMode() {


    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    try {
      runSample();
    } finally {

      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.WHITE);
        }
      });
      }
  }

  protected void runSample() {

    float gain = 2;


    final float[] hsvValues = new float[3];


    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;


    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }


    waitForStart();


    while (opModeIsActive()) {
          if (gamepad1.a) {

        gain += 0.005;
      } else if (gamepad1.b && gain > 1) {
        gain -= 0.005;
      }


      telemetry.addData("Gain", gain);

           colorSensor.setGain(gain);


      xButtonCurrentlyPressed = gamepad1.x;


      if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {

        if (xButtonCurrentlyPressed) {
          if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(!light.isLightOn());
          }
        }
      }
      xButtonPreviouslyPressed = xButtonCurrentlyPressed;


      NormalizedRGBA colors = colorSensor.getNormalizedColors();
      double red = colors.red;
      double blue = colors.blue;
      double green = colors.green;
      String mainColor;
      String setColor = "red";
      boolean CorrectColor;
      if( red > blue && red > green){
        mainColor = "red";
      }else if ( blue > red && blue > green){
        mainColor = "blue";
      }else {
        mainColor = "yellow";
      }
      if(mainColor == setColor || mainColor == "yellow"){
        CorrectColor = true;
      }else{
        CorrectColor = false;
      }




      telemetry.addLine()
              .addData("color", mainColor)
              .addData("match", CorrectColor)
              .addData("Red", "%.3f", colors.red)
              .addData("Green", "%.3f", colors.green)
              .addData("Blue", "%.3f", colors.blue);
      telemetry.addLine()
              .addData("Hue", "%.3f", hsvValues[0])
              .addData("Saturation", "%.3f", hsvValues[1])
              .addData("Value", "%.3f", hsvValues[2]);
      telemetry.addData("Alpha", "%.3f", colors.alpha);

      if (colorSensor instanceof DistanceSensor) {
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
      }

      telemetry.update();


      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
        }
      });
    }
  }
}

