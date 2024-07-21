package org.firstinspires.ftc.teamcode.PathingTool;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Commands.ArmDown;
import org.firstinspires.ftc.teamcode.Commands.ArmUp;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.ElevatorDown;
import org.firstinspires.ftc.teamcode.Commands.ElevatorUp;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

public class PathEngine {

    private JSONObject jsonPathData;
    private static int currentIndex = 0;
    private final double COORDINATE_UNIT = 1.0;
    private double[] lastPoint = {0.0, 0.0};
    private double[] currentPoint = {0.0, 0.0};
    private double startTime = 0.0;
    private CommandScheduler scheduler = CommandScheduler.getInstance();
    private HashMap<String, Command> commandMap = new HashMap<>();

    public PathEngine(OpMode opMode, String pathFileName) {
        loadJSONFromAsset(opMode, pathFileName);
        initializeCommandMap();
        System.out.println(jsonPathData);
    }

    private void loadJSONFromAsset(OpMode opMode, String pathFileName) {
        try {
            BufferedReader reader = new BufferedReader(
                    new InputStreamReader(opMode.hardwareMap.appContext.getAssets().open(pathFileName))
            );
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }
            reader.close();
            jsonPathData = new JSONObject(sb.toString());
        } catch (IOException | JSONException e) {
            e.printStackTrace();

        }
    }

    private void initializeCommandMap() {
        commandMap.put("Outtake", new Outtake());
        commandMap.put("Intake", new Intake());
        commandMap.put("ElevatorDown", new ElevatorDown());
        commandMap.put("ElevatorUp", new ElevatorUp());
        commandMap.put("ArmUp", new ArmUp());
        commandMap.put("ArmDown", new ArmDown());
        commandMap.put("SequentialCommandGroup", new SequentialCommandGroup());
    }

    public void startPath(double startTime) {
        this.startTime = startTime;
    }

    public double getElapsedTime(double currentTime) {
        return currentTime - startTime;
    }

    public void update(double currentTime) throws InterruptedException, JSONException {
        double elapsedTime = getElapsedTime(currentTime);
        if (currentIndex < jsonPathData.getJSONArray("key_points").length()) {
            JSONObject currentPointData = jsonPathData.getJSONArray("key_points").getJSONObject(currentIndex);
            double pointTime = currentPointData.getDouble("time");

            if (elapsedTime >= pointTime) {
                double x = currentPointData.getDouble("x") * COORDINATE_UNIT;
                double y = currentPointData.getDouble("y") * COORDINATE_UNIT;
                double angle = currentPointData.getDouble("angle");

                DriveSubsystem.moveToPosition(angle, x, y);

                lastPoint[0] = currentPoint[0];
                lastPoint[1] = currentPoint[1];
                currentPoint[0] = x;
                currentPoint[1] = y;

                JSONArray commands = jsonPathData.optJSONArray("commands");
                if (commands != null) {
                    for (int i = 0; i < commands.length(); i++) {
                        JSONObject commandData = commands.getJSONObject(i);
                        double commandStart = commandData.getDouble("start");
                        double commandEnd = commandData.getDouble("end");
                        String commandName = commandData.getJSONObject("command").getString("name");

                        if (elapsedTime >= commandStart && elapsedTime <= commandEnd) {
                            Command command = commandMap.get(commandName);
                            if (command != null) {
                                scheduler.schedule(command);
                            }
                        }
                    }
                }

                currentIndex++;
            }
        }
    }

    public double calculateDistance() {
        double lastX = lastPoint[0];
        double lastY = lastPoint[1];
        double currentX = currentPoint[0];
        double currentY = currentPoint[1];

        double deltaX = currentX - lastX;
        double deltaY = currentY - lastY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
}
