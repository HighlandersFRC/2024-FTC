
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PWMOutputImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ElevatorDefault;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.IntakeDefault;
import org.firstinspires.ftc.teamcode.Tools.Robot;

public class Intake extends Subsystem{

    private static NormalizedColorSensor colorSensor;
    public static CRServo leftServo;
    public static CRServo rightServo;

    private static CRServoImplEx left, right;
    private static final String setColor = "blue";

    public Intake(String name) {
        super(name);
    }


    public static void initialize(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");

/*
        left = hardwareMap.get(CRServoImplEx.class, "left_servo");
        right = hardwareMap.get(CRServoImplEx.class, "right_servo");

        left.isPwmEnabled();
        right.isPwmEnabled();

        left.setPwmRange(new PwmControl.PwmRange(500,2500));
        right.setPwmRange(new PwmControl.PwmRange(500,2500));
*/

        Intake.stopIntake();
    }

    public void intake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
/*
        left.setPower(-1);
        right.setPower(1);
*/
    }

    public void outtake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);

        /*        left.setPower(1);
        right.setPower(-1);
 */   }
    public void outtakePower(double Left, double Right){
        leftServo.setPower(Left);
        rightServo.setPower(-Right);
    }

    public static void stopIntake()  {
       /* try {
            leftServo.setPower(0.05);
            rightServo.setPower(-0.05);
            Thread.sleep(50);*/

/*
        leftServo.setPower(0);
*/      leftServo.setPower(0);
        rightServo.setPower(0);
       /* } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }*/
    }

    public boolean getCorrectColor() {
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
    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(new IntakeDefault(Robot.intake));
    }

    @Override
    public Command getDefaultCommand() {
        return new IntakeDefault(Robot.intake);
    }
}