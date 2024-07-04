package org.firstinspires.ftc.teamcode.PathingTool;

import android.content.Context;
import android.content.res.AssetManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.TestAuto;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;

public class PathEngine {

    private String pathFileName;
    private DriveSubsystem driveSubsystem;
    private ArrayList<PathPoint> pathPoints;
    private Telemetry telemetry;
    private TestAuto context;

    public PathEngine(String pathFileName, DriveSubsystem driveSubsystem, Telemetry telemetry, TestAuto context) {
        this.pathFileName = pathFileName;
        this.driveSubsystem = driveSubsystem;
        this.telemetry = telemetry;
        this.context = context;
    }

    public void initializePath() throws IOException, JSONException {
        JSONArray jsonArray = loadJSONArrayFromAssets(pathFileName);

        pathPoints = new ArrayList<>();
        for (int i = 0; i < jsonArray.length(); i++) {
            JSONObject jsonObject = jsonArray.getJSONObject(i);
            double x = jsonObject.getDouble("x");
            double y = jsonObject.getDouble("y");
            pathPoints.add(new PathPoint(x, y));
        }
    }

    public void runPathFollowing() throws InterruptedException {
        for (PathPoint point : pathPoints) {
            double targetX = point.getX();
            double targetY = point.getY();

            // Example movement logic (replace with your own)
            driveSubsystem.moveToPoint(targetX, targetY);

            // Example telemetry updates (replace with your own)
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.update();

            Thread.sleep(1000); // Example delay (adjust as needed)
        }
    }


    private JSONArray loadJSONArrayFromAssets(String fileName) throws IOException, JSONException {
        AssetManager assetManager = context.getAssets();
        InputStream inputStream = assetManager.open(fileName);
        BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
        StringBuilder builder = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            builder.append(line);
        }
        reader.close();

        return new JSONArray(builder.toString());
    }
}
