package org.firstinspires.ftc.teamcode.PathingTool;

import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PolarPathFollower implements Command {

    private Set<String> addedCommandKeys = new HashSet<>();
    private CommandScheduler scheduler;
    private double pathStartTime;
    private JSONArray points;

    private PID xPID = new PID(2, 0, 0);
    private PID yPID = new PID(2, 0, 0);
    private PID yawPID = new PID(5, 0, 0);
    double nextX;
    double nextY;


    public PolarPathFollower(Drive drive, Peripherals peripherals, JSONObject pathJSON,
                             HashMap<String, Supplier<Command>> commandMap,
                             HashMap<String, BooleanSupplier> conditionMap,
                             CommandScheduler scheduler) throws JSONException {
        super();
        this.scheduler = scheduler;
        this.points = pathJSON.getJSONArray("sampled_points");
        JSONArray commands = pathJSON.getJSONArray("commands");
        for (int i = 0; i < commands.length(); i++) {
            JSONObject command = commands.getJSONObject(i);
            String commandKey = command.toString();
            if (!addedCommandKeys.contains(commandKey)) {
                Command newCommand = addCommandsFromJSON(command, commandMap, conditionMap);
                if (newCommand != null) {
                    scheduler.schedule(newCommand);
                    addedCommandKeys.add(commandKey);
                }
            }
        }
    }

    private Command addCommandsFromJSON(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
                                        HashMap<String, BooleanSupplier> conditionMap) throws JSONException {
        if (command.has("command")) {
            return createTriggerCommand(command, commandMap);
        } else if (command.has("branched_command")) {
            return createBranchedCommand(command, commandMap, conditionMap);
        } else if (command.has("parallel_command_group")) {
            return createParallelCommandGroup(command, commandMap, conditionMap);
        } else if (command.has("parallel_deadline_group")) {
            return createParallelDeadlineGroup(command, commandMap, conditionMap);
        } else if (command.has("parallel_race_group")) {
            return createParallelRaceGroup(command, commandMap, conditionMap);
        } else if (command.has("sequential_command_group")) {
            return createSequentialCommandGroup(command, commandMap, conditionMap);
        } else {
            throw new IllegalArgumentException("Invalid command JSON: " + command.toString());
        }
    }

    private TriggerCommand createTriggerCommand(JSONObject command, HashMap<String, Supplier<Command>> commandMap) throws JSONException {
        BooleanSupplier startSupplier = () -> {
            try {
                return command.getDouble("start") < getPathTime();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
        };
        BooleanSupplier endSupplier = () -> {
            try {
                return command.getDouble("end") <= getPathTime();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
        };
        return new TriggerCommand(scheduler, startSupplier, commandMap.get(command.getJSONObject("command").getString("name")).get(), endSupplier);
    }

    private Command createBranchedCommand(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
                                          HashMap<String, BooleanSupplier> conditionMap) throws JSONException {
        BooleanSupplier startSupplier = () -> {
            try {
                return command.getDouble("start") < getPathTime();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
        };
        BooleanSupplier endSupplier = () -> {
            try {
                return command.getDouble("end") <= getPathTime();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
        };
        JSONObject branchedCommand = command.getJSONObject("branched_command");
        BooleanSupplier condition = conditionMap.get(branchedCommand.getString("condition"));
        return new TriggerCommand(
                scheduler,
                startSupplier,
                new ConditionalCommand(
                        condition,
                        createTriggerCommand(branchedCommand.getJSONObject("on_true"), commandMap),
                        createTriggerCommand(branchedCommand.getJSONObject("on_false"), commandMap)
                ),
                endSupplier
        );
    }

    private ParallelCommandGroup createParallelCommandGroup(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
                                                            HashMap<String, BooleanSupplier> conditionMap) throws JSONException {
        ParallelCommandGroup parallelGroup = new ParallelCommandGroup(scheduler, Parameters.ALL);
        JSONArray parallelCommands = command.getJSONObject("parallel_command_group").getJSONArray("commands");
        for (int i = 0; i < parallelCommands.length(); i++) {
            parallelGroup.addCommands(addCommandsFromJSON(parallelCommands.getJSONObject(i), commandMap, conditionMap));
        }
        return parallelGroup;
    }

    private ParallelCommandGroup createParallelDeadlineGroup(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
                                                             HashMap<String, BooleanSupplier> conditionMap) throws JSONException {
        ParallelCommandGroup parallelGroup = new ParallelCommandGroup(scheduler, Parameters.SPECIFIC,
                addCommandsFromJSON(command.getJSONObject("parallel_deadline_group").getJSONArray("commands").getJSONObject(0), commandMap, conditionMap));
        for (int i = 1; i < command.getJSONObject("parallel_deadline_group").getJSONArray("commands").length(); i++) {
            parallelGroup.addCommands(addCommandsFromJSON(
                    command.getJSONObject("parallel_deadline_group").getJSONArray("commands").getJSONObject(i), commandMap, conditionMap));
        }
        return parallelGroup;
    }

    private ParallelCommandGroup createParallelRaceGroup(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
                                                         HashMap<String, BooleanSupplier> conditionMap) throws JSONException {
        ParallelCommandGroup parallelGroup = new ParallelCommandGroup(scheduler, Parameters.ANY);
        JSONArray raceCommands = command.getJSONObject("parallel_race_group").getJSONArray("commands");
        for (int i = 0; i < raceCommands.length(); i++) {
            parallelGroup.addCommands(addCommandsFromJSON(raceCommands.getJSONObject(i), commandMap, conditionMap));
        }
        return parallelGroup;
    }

    private SequentialCommandGroup createSequentialCommandGroup(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
                                                                HashMap<String, BooleanSupplier> conditionMap) throws JSONException {
        SequentialCommandGroup sequentialGroup = new SequentialCommandGroup(scheduler);
        JSONArray sequentialCommands = command.getJSONObject("sequential_command_group").getJSONArray("commands");
        for (int i = 0; i < sequentialCommands.length(); i++) {
            sequentialGroup.addCommands(addCommandsFromJSON(sequentialCommands.getJSONObject(i), commandMap, conditionMap));
        }
        return sequentialGroup;
    }

    private double getPathTime() {
        return System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void start() {
        this.pathStartTime  = getPathTime();

        try{
        JSONObject currentPoint = points.getJSONObject(0);
        nextX = currentPoint.getDouble("x");
        nextY = currentPoint.getDouble("y");
        double nextTheta = currentPoint.getDouble("angle");

        Mouse.setPosition(nextX, nextY, Math.toDegrees(nextTheta));

        }  catch (JSONException e) {
            throw new RuntimeException("Error reading point data from JSON", e);
        }

        yawPID.setMinInput(-180);
        yawPID.setMinInput(180);
    }

    public void execute() {


        FinalPose.poseUpdate();
        double elapsedTime = getPathTime() - pathStartTime;
        //look ahead distance of 50 ms
        int index = (int) ((elapsedTime + 0.05) / 0.01);
        if (index >= points.length()) {
            index = points.length() - 1;
        }
        try {
            JSONObject currentPoint = points.getJSONObject(index);
            nextX = currentPoint.getDouble("x");
            nextY = currentPoint.getDouble("y");
            double nextTheta = currentPoint.getDouble("angle");
            double currentX = FinalPose.x;
            double currentY = FinalPose.y;
            double currentTheta = Math.toRadians(FinalPose.Yaw);
            double relativeX = nextX - currentX;
            double relativeY = nextY - currentY;
            double relativeTheta = nextTheta - currentTheta;

            xPID.setSetPoint(nextX);
            xPID.updatePID(currentX);

            yPID.setSetPoint(nextY);
            yPID.updatePID(currentY);

            yawPID.setSetPoint(nextTheta);
            yawPID.updatePID(currentTheta);

            Vector relativePos = new Vector(xPID.getResult(), yPID.getResult());
            Drive.autoDrive(relativePos, yawPID.getResult());

            System.out.println("Vector X" + relativePos.getI() + "Vector Y " + relativePos.getJ() + "Theta " + relativeTheta + "Index " + index + "     Current X " + currentX + "    Current Y " + currentY + "    Current Theta " +  currentTheta + "   Next Point X   " + nextX + "      Next Point Y     " + nextY + "    Next Point Theta     " + nextTheta);

        } catch (JSONException e) {
            throw new RuntimeException("Error reading point data from JSON", e);
        }
    }

    @Override
    public void end() {
        Drive.stop();
    }

    @Override
    public boolean isFinished() {
        try{
            double elapsedTime = getPathTime() - pathStartTime;
            int index = (int) (elapsedTime / 0.01);

            JSONObject currentPoint = points.getJSONObject(points.length() - 1);
            double finalX = currentPoint.getDouble("x");
            double finalY = currentPoint.getDouble("y");
            double finalTheta = currentPoint.getDouble("angle");

            if (index >= points.length() && (findDistance(FinalPose.x, FinalPose.y, finalX, finalY ) <= 0.02) && (Math.abs(finalTheta) - Math.abs(FinalPose.Yaw)) <= Math.toRadians(3)) {
                return true;
            }
        }
        catch (JSONException e){
            System.out.println("Error Parsing Json");
        }
        return false;
    }
    public double findDistance(double currentX, double currentY, double targetX, double targetY){
        return (Math.sqrt(Math.pow(targetY - currentY, 2) + Math.pow(targetX - currentX, 2)));
    }
}
