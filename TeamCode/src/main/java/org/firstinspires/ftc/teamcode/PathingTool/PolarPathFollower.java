package org.firstinspires.ftc.teamcode.PathingTool;

import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
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

import org.firstinspires.ftc.teamcode.Tools.Mouse;


public class PolarPathFollower extends SequentialCommandGroup {

    private Set<String> addedCommandKeys = new HashSet<>();
    private CommandScheduler scheduler;
    private double pathStartTime;
    private JSONArray points;

    public PolarPathFollower(Drive drive, Peripherals peripherals, JSONObject pathJSON,
                             HashMap<String, Supplier<Command>> commandMap,
                             HashMap<String, BooleanSupplier> conditionMap,
                             CommandScheduler scheduler) throws JSONException {
        super(scheduler);
        this.scheduler = scheduler;
        this.pathStartTime = getPathTime();
        this.points = pathJSON.getJSONArray("sampled_points");

        JSONArray commands = pathJSON.getJSONArray("commands");
        for (int i = 0; i < commands.length(); i++) {
            JSONObject command = commands.getJSONObject(i);
            String commandKey = command.toString();

            if (!addedCommandKeys.contains(commandKey)) {
                Command newCommand = addCommandsFromJSON(command, commandMap, conditionMap);
                if (newCommand != null) {
                    addCommands(newCommand);
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
        ParallelCommandGroup parallelGroup = new ParallelCommandGroup(scheduler, Parameters.SPECIFIC, addCommandsFromJSON(
                command.getJSONObject("parallel_deadline_group").getJSONArray("commands").getJSONObject(0),
                commandMap, conditionMap));

        for (int i = 1; i < command.getJSONObject("parallel_deadline_group").getJSONArray("commands").length(); i++) {
            parallelGroup.addCommands(addCommandsFromJSON(
                    command.getJSONObject("parallel_deadline_group").getJSONArray("commands").getJSONObject(i),
                    commandMap, conditionMap));
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


    private double LOOK_AHEAD_TIME = 0.1;
    private double TIME_STEP = 0.01;

    private PID positionPID = new PID(0.4, 0.0, 0.0);
    private PID anglePID = new PID(0.2, 0.0, 0.0);

    public void execute() {
        Mouse.update();
        double elapsedTime = getPathTime() - pathStartTime + LOOK_AHEAD_TIME;
        int index = Math.min((int) (elapsedTime / TIME_STEP), points.length() - 1);

        try {
            JSONObject currentPoint = points.getJSONObject(index);
            Constants.nextX = currentPoint.getDouble("x");
            Constants.nextY = currentPoint.getDouble("y");
            Constants.nextTheta = currentPoint.getDouble("angle");

            double currentX = FinalPose.getX();
            double currentY = FinalPose.getY();
            double currentTheta = FinalPose.getYaw();

            double targetX = Constants.nextX;
            double targetY = Constants.nextY;
            double targetTheta = Constants.nextTheta;

            double relativeX = targetX - currentX;
            double relativeY = targetY - currentY;
            double positionError = Math.hypot(relativeX, relativeY);
            double angularError = targetTheta - currentTheta;
            angularError = Math.atan2(Math.sin(angularError), Math.cos(angularError));

            positionPID.setSetPoint(0);
            double positionCorrection = positionPID.updatePID(positionError);

            anglePID.setSetPoint(0);
            double angularCorrection = anglePID.updatePID(angularError);

            Vector direction = new Vector(relativeX, relativeY).normalize().scale(positionCorrection);

            Drive.autoDrive(direction, angularCorrection);
        } catch (JSONException e) {
            System.err.println("Error reading point data from JSON: " + e.getMessage());
        }
    }


}
