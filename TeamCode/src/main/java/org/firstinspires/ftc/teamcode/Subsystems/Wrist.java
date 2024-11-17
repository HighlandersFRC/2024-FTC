package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist extends Subsystem  {
    public static CRServo wrist;

    public static void initialize(HardwareMap hardwareMap) {
        wrist = hardwareMap.crservo.get("intake");
    }
    public static void setPower(double power){
        wrist.setPower(power);
    }
}
