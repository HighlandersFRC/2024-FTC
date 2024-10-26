package org.firstinspires.ftc.teamcode.PathingTool;

import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
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

public class PolarPathFollower extends SequentialCommandGroup {

    private Set<String> addedCommandKeys = new HashSet<>();
    private CommandScheduler scheduler;

    public PolarPathFollower(Drive drive, Peripherals peripherals, JSONObject pathJSON,
                             HashMap<String, Supplier<Command>> commandMap,
                             HashMap<String, BooleanSupplier> conditionMap,
                             CommandScheduler scheduler) throws JSONException {
        super(scheduler);
        this.scheduler = scheduler;

        JSONArray points = pathJSON.getJSONArray("sampled_points");
        JSONArray commands = pathJSON.getJSONArray("commands");

        for (int i = 0; i < points.length(); i++) {
            JSONObject point = points.getJSONObject(i);
            double x = point.getDouble("x");
            double y = point.getDouble("y");
            double theta = point.getDouble("angle");
            System.out.println("x: " + x + " y: " + y);

/*
            Vector pointVector = Drive.purePursuitController(FinalPose.x, FinalPose.y, FinalPose.Yaw, i, points);
*/

            MoveToPosition moveToPosition = new MoveToPosition(x, y, theta);

            addCommands(moveToPosition);
        }

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
}
