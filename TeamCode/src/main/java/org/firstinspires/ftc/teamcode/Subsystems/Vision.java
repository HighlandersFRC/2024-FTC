package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision extends Subsystem{
    Limelight3A limeLight;
    private static String name = "Vision";
    public static void initialize(HardwareMap hardwareMap){
        //Everything that runs once
    }

    public Vision(String name) {
        super(name);
    }

    public static void update(){
        // Update the vision data here

    }
        //getter methods
}