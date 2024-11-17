package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends Subsystem  {
    public static CRServo intake;
    public IntakeSubsystem(String name) {
        super();
    }
    public static void initialize(HardwareMap hardwareMap) {
        intake = hardwareMap.crservo.get("intake");
    }
    public static void setPower(double power){
        intake.setPower(power);
    }
}
