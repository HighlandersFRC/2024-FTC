/*
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class IntakeSubsystem extends Subsystem {

    private NormalizedColorSensor colorSensor = null;
    public static CRServo leftServo;
    public static CRServo rightServo;
    private final String setColor = "red";

    public void Intake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }
public static void intake(){
        leftServo.setPower(1);
        rightServo.setPower(-1);
}
    public static void outtake(){
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }
    public boolean isCorrectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor;
        if (red > blue && red > green) {
            mainColor = "red";
        } else if (blue > red && blue > green) {
            mainColor = "blue";
        } else {
            mainColor = "yellow";
        }

        return mainColor.equals(setColor) || mainColor.equals("yellow");

    }

}
*//*
*/
/*

package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem extends Subsystem {

    private NormalizedColorSensor colorSensor = null;
    public static CRServo leftServo;
    public static CRServo rightServo;
    private final String setColor = "red";

    public void Intake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    public static void intake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);
    }

    public static void outtake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }
    public String getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green) {
            mainColor = "red";
        } else if (blue > red && blue > green) {
            mainColor = "blue";
        } else if (red > blue && green > blue){
            mainColor = "yellow";
        }

        return mainColor;
    }

    public boolean isCorrectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green && red > 0.01) {
            mainColor = "red";
        } else if (blue > red && blue > green && blue > 0.01) {
            mainColor = "blue";
        } else if(red > blue && green > blue && red > 0.01 && green > 0.01){
            mainColor = "yellow";
        }

        return mainColor.equals(setColor) || mainColor.equals("yellow");
    }


    public void stopIntake() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}*//*

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class IntakeSubsystem extends Subsystem {

    private NormalizedColorSensor colorSensor = null;
    public static CRServo leftServo;
    public static CRServo rightServo;
    private final String setColor = "red";

    public void Intake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    public static void intake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);
    }

    public static void outtake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }

    public String getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green) {
            mainColor = "red";
        } else if (blue > red && blue > green) {
            mainColor = "blue";
        } else if (red > blue && green > blue) {
            mainColor = "yellow";
        }

        return mainColor;
    }

    public boolean isCorrectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green && red > 0.01) {
            mainColor = "red";
        } else if (blue > red && blue > green && blue > 0.01) {
            mainColor = "blue";
        } else if (red > blue && green > blue && red > 0.01 && green > 0.01) {
            mainColor = "yellow";
        }

        return mainColor.equals(setColor) || mainColor.equals("yellow");
    }

    public void intakeSample() {

        intake();


        while (!getDetectedColor().equals(setColor)) {

            intake();
        }


        stopIntake();
    }

    public void stopIntake() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}
*/package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.Tools.CustomServo.NewServo;

public class IntakeSubsystem extends Subsystem {

    private NormalizedColorSensor colorSensor = null;
    public static CRServo leftServo ;
    public static CRServo rightServo;
    private final String setColor = "red";
    private boolean intakeStarted = false;
    private boolean isStopped = false;

    public void Intake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }


    }

    public static void intake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);

    }

    public static void outtake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }

    public String getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green) {
            mainColor = "red";
        } else if (blue > red && blue > green) {
            mainColor = "blue";
        } else if (red > blue && green > blue) {
            mainColor = "yellow";
        }

        return mainColor;
    }

    public boolean isCorrectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green && red > 0.01) {
            mainColor = "red";
        } else if (blue > red && blue > green && blue > 0.01) {
            mainColor = "blue";
        } else if (red > blue && green > blue && red > 0.01 && green > 0.01) {
            mainColor = "yellow";
        }

        return mainColor.equals(setColor) || mainColor.equals("yellow");
    }

    public void intakeSample() {
        isStopped = false;

        while (!getDetectedColor().equals(setColor) && !getDetectedColor().equals("yellow")) {
            if (isStopped) {
                stopIntake();
                return;
            }
            intake();
        }

        stopIntake();
    }

    public void stopIntake() {
        isStopped = true;
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}
