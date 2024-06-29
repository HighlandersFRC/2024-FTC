package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Peripherals extends Subsystem{

    static IMU imu;

    public Peripherals(String name) {
        super(name);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

    }

    public static void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
    }

    public static double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public static double getRoll(){
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public static double getPitch(){
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public static void resetYaw(){
        imu.resetYaw();
    }
}
