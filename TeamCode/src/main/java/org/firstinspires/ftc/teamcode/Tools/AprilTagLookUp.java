package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilTagLookUp {

    public static AprilTagLibrary getSmallLibrary() {
        return new AprilTagLibrary.Builder()
                .setAllowOverwrite(true)
                .addTag(7,
                        "tag 14",
                        0.127,//test
                        new VectorF(0,0),
                        DistanceUnit.METER,
                        Quaternion.identityQuaternion())

                .build();

    }
}
