package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;

public class TrajectoryLoader {

    public static Trajectory loadTrajectory(JSONObject json, double dragCoeff, double magnusCoeff, double magnusPower) {
        try {
            double exitAngleDeg = json.getDouble("exitAngle");
            double speed = json.getDouble("speed");
            double timeOfFlight = json.getDouble("tof");
            double speedMoe = optDouble(json, 0.0, "speedMOE", "speedMoe");
            double angleMoeDeg = optDouble(json, 0.0, "angleMOE", "angleMoe");

            return new Trajectory(
                    dragCoeff,
                    magnusCoeff,
                    magnusPower,
                    speed,
                    Math.toRadians(exitAngleDeg),
                    timeOfFlight,
                    speedMoe,
                    Math.toRadians(angleMoeDeg),
                    true
            );
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static TrajectoryLUT loadTrajectoryLUT(JSONObject groupJson, double dy, double dragCoeff, double magnusCoeff, double magnusPower) {
        try {
            if (!groupJson.has("dx"))
                return null;

            double dx = groupJson.getDouble("dx");
            JSONArray trajectoryArray = groupJson.getJSONArray("trajectories");
            ArrayList<Trajectory> trajectories = new ArrayList<>();

            for (int i = 0; i < trajectoryArray.length(); i++) {
                JSONObject trajJson = trajectoryArray.getJSONObject(i);
                Trajectory trajectory = loadTrajectory(trajJson, dragCoeff, magnusCoeff, magnusPower);
                if (trajectory == null)
                    return null;
                trajectories.add(trajectory);
            }

            if (trajectories.isEmpty())
                return null;

            int optimalIndex = resolveOptimalTrajectoryIndex(groupJson, trajectories);
            if (optimalIndex < 0 || optimalIndex >= trajectories.size())
                return null;

            return new TrajectoryLUT(
                    dx,
                    dy,
                    dragCoeff,
                    magnusCoeff,
                    optimalIndex,
                    trajectories
            );
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static TrajectoryDistanceLUT loadTrajectoryDistanceLUT(JSONObject root) {
        try {
            double dy = root.getDouble("dy");
            double dragCoeff = root.getDouble("dragCoeff");
            double magnusCoeff = root.getDouble("magnusCoeff");
            double magnusPower = root.getDouble("magnusPower");
            JSONArray groups = root.getJSONArray("groups");
            ArrayList<TrajectoryLUT> trajectoryLUTs = new ArrayList<>();

            for (int i = 0; i < groups.length(); i++) {
                TrajectoryLUT trajectoryLUT = loadTrajectoryLUT(groups.getJSONObject(i), dy, dragCoeff, magnusCoeff, magnusPower);
                if (trajectoryLUT != null)
                    trajectoryLUTs.add(trajectoryLUT);
            }

            if (trajectoryLUTs.isEmpty())
                throw new RuntimeException("No trajectory groups loaded from JSON");

            return TrajectoryDistanceLUT.fromTrajectoryLUTs(trajectoryLUTs);
        } catch (JSONException e) {
            throw new RuntimeException("Failed to parse trajectory groups from JSON", e);
        }
    }

    // returns the trajectoryLUT with the distance closest to distFromGoalMeters
    public static TrajectoryLUT loadTrajectoryLUT(String filename, double distFromGoalMeters) {
        JSONObject root = getJsonObject(filename);
        try {
            double dy = root.getDouble("dy");
            double dragCoeff = root.getDouble("dragCoeff");
            double magnusCoeff = root.getDouble("magnusCoeff");
            double magnusPower = root.getDouble("magnusPower");
            JSONArray groups = root.getJSONArray("groups");

            double closestDistError = -1;
            JSONObject closestGroup = null;
            for (int i=0; i<groups.length(); i++) {
                JSONObject group = groups.getJSONObject(i);
                double distError = Math.abs(group.getDouble("dx") - distFromGoalMeters);
                if (closestDistError == -1 || distError < closestDistError) {
                    closestDistError = distError;
                    closestGroup = group;
                    if (distError == 0)
                        break;
                }
            }
            if (closestGroup == null)
                    return null;

            return loadTrajectoryLUT(closestGroup, dy, dragCoeff, magnusCoeff, magnusPower);
        } catch (JSONException e) {
            throw new RuntimeException("Failed to load trajectory group from " + filename + " with closest dx to " + distFromGoalMeters + "meters", e);
        }
    }

    public static TrajectoryDistanceLUT loadTrajectoryDistanceLUT(String filename) {
        return loadTrajectoryDistanceLUT(getJsonObject(filename));
    }

    public static JSONObject getJsonObject(String filepath) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filepath);
            String contents = ReadWriteFile.readFile(file);
            return new JSONObject(contents);
        } catch (Exception e) {
            throw new RuntimeException("Failed to read JSON file: " + filepath, e);
        }
    }

    private static int resolveOptimalTrajectoryIndex(JSONObject groupJson, ArrayList<Trajectory> trajectories) {
        if (groupJson.has("optimalTrajectoryIndex"))
            return groupJson.optInt("optimalTrajectoryIndex");
        if (groupJson.has("biggestMOETrajectory"))
            return groupJson.optInt("biggestMOETrajectory");

        int bestIndex = 0;
        double bestMoe = -1.0;
        for (int i = 0; i < trajectories.size(); i++) {
            Trajectory trajectory = trajectories.get(i);
            double combinedMoe = trajectory.exitSpeedMOE * trajectory.exitAngleMOERad;
            if (combinedMoe > bestMoe) {
                bestMoe = combinedMoe;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    private static double optDouble(JSONObject json, double defaultValue, String... keys) throws JSONException {
        for (String key : keys) {
            if (json.has(key))
                return json.getDouble(key);
        }
        return defaultValue;
    }
}
