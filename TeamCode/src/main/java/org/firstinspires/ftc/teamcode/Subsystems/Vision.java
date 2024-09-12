package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision extends Subsystem{
    private static String name = "Vision";
   Limelight3A Limelight3A = new Limelight3A();
    public static void initialize(HardwareMap hardwaremap) {

    }
    public static void update(HardwareMap hardwaremap){}
}
