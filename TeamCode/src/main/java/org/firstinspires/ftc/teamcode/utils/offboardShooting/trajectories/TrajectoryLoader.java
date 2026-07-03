package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;

public class TrajectoryLoader {

    public static Trajectory loadTrajectory(JSONObject json, double dragCoeff, double magnusCoeff, double magnusPower) {
        try {
            if (!json.has("exitAngle"))
                throw new IllegalArgumentException("trajectory json does not have exit angle: " + json);
            if (!json.has("speed"))
                throw new IllegalArgumentException("trajectory json does not have speed: " + json);
            if (!json.has("tof"))
                throw new IllegalArgumentException("trajectory json does not have tof: " + json);
            if (!json.has("speedMOE"))
                throw new IllegalArgumentException("trajectory json does not have speedMOE: " + json);
            if (!json.has("angleMOE"))
                throw new IllegalArgumentException("trajectory json does not have angleMOE: " + json);

            double exitAngleDeg = json.getDouble("exitAngle");
            double speed = json.getDouble("speed");
            double timeOfFlight = json.getDouble("tof");
            double speedMoe = json.getDouble("speedMOE");
            double angleMoeDeg = json.getDouble("angleMOE");

            return new Trajectory(
                    dragCoeff,
                    magnusCoeff,
                    magnusPower,
                    speed,
                    Angle2d.fromDegrees(exitAngleDeg),
                    timeOfFlight,
                    speedMoe,
                    Angle2d.fromDegrees(angleMoeDeg),
                    true
            );
        } catch (JSONException e) {
            e.printStackTrace();
            throw new RuntimeException("JSON exception when calling loadTrajectory");
        }
    }

    public static TrajectoryLUT loadTrajectoryLUT(JSONObject groupJson, double dy, double dragCoeff, double magnusCoeff, double magnusPower) {
        try {
            if (!groupJson.has("dx"))
                throw new IllegalArgumentException("groupJSON is invalid. no dx key. json: " + groupJson);
            if (!groupJson.has("trajectories"))
                throw new IllegalArgumentException("groupJSON is invalid. no trajectories key. json: " + groupJson);
            if (!groupJson.has("optimalHighArcTrajectoryIndex"))
                throw new IllegalArgumentException("groupJSON is invalid. no optimalHighArcTrajectoryIndex key. json: " + groupJson);
            if (!groupJson.has("optimalLowArcTrajectoryIndex"))
                throw new IllegalArgumentException("groupJSON is invalid. no optimalLowArcTrajectoryIndex key. json: " + groupJson);

            double dx = groupJson.getDouble("dx");
            JSONArray trajectoryArray = groupJson.getJSONArray("trajectories");
            ArrayList<Trajectory> trajectories = new ArrayList<>();

            for (int i = 0; i < trajectoryArray.length(); i++) {
                JSONObject trajJson = trajectoryArray.getJSONObject(i);
                Trajectory trajectory = loadTrajectory(trajJson, dragCoeff, magnusCoeff, magnusPower);
                trajectories.add(trajectory);
            }

            if (trajectories.isEmpty())
                return null;

            int optimalHighArcIndex = groupJson.getInt("optimalHighArcTrajectoryIndex");
            int optimalLowArcIndex = groupJson.getInt("optimalLowArcTrajectoryIndex");
            if (optimalHighArcIndex < 0 || optimalHighArcIndex >= trajectories.size())
                throw new IllegalArgumentException("JSON file has invalid optimal high arc index at " + dx + "meters. " + optimalHighArcIndex + " must be >= 0 and < " + trajectories.size());
            if (optimalLowArcIndex < 0 || optimalLowArcIndex >= trajectories.size())
                throw new IllegalArgumentException("JSON file has invalid optimal low arc index at " + dx + "meters. " + optimalLowArcIndex + " must be >= 0 and < " + trajectories.size());

            return new TrajectoryLUT(
                    dx,
                    dy,
                    dragCoeff,
                    magnusCoeff,
                    optimalHighArcIndex,
                    optimalLowArcIndex,
                    trajectories
            );
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static TrajectoryDistanceLUT loadTrajectoryDistanceLUT(JSONObject root) {
        try {
            if (!root.has("dy"))
                throw new IllegalArgumentException("trajectoryDistanceLUT json does not have dy: " + root);
            if (!root.has("dragCoeff"))
                throw new IllegalArgumentException("trajectoryDistanceLUT json does not have dragCoeff: " + root);
            if (!root.has("magnusCoeff"))
                throw new IllegalArgumentException("trajectoryDistanceLUT json does not have magnusCoeff: " + root);
            if (!root.has("magnusPower"))
                throw new IllegalArgumentException("trajectoryDistanceLUT json does not have magnusPower: " + root);
            if (!root.has("groups"))
                throw new IllegalArgumentException("trajectoryDistanceLUT json does not have groups: " + root);

            double dy = root.getDouble("dy");
            double dragCoeff = root.getDouble("dragCoeff");
            double magnusCoeff = root.getDouble("magnusCoeff");
            double magnusPower = root.getDouble("magnusPower");
            JSONArray groups = root.getJSONArray("groups");
            ArrayList<TrajectoryLUT> trajectoryLUTs = new ArrayList<>();

            for (int i = 0; i < groups.length(); i++) {
                JSONObject trajectoryLUTJson = groups.getJSONObject(i);
                TrajectoryLUT trajectoryLUT = loadTrajectoryLUT(trajectoryLUTJson, dy, dragCoeff, magnusCoeff, magnusPower);
                if (trajectoryLUT == null)
                    throw new IllegalStateException("trajectoryLUT is null when loading from JSONObject root. trajectoryLUT json: " + trajectoryLUTJson);
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

    private static double optDouble(JSONObject json, double defaultValue, String... keys) throws JSONException {
        for (String key : keys) {
            if (json.has(key))
                return json.getDouble(key);
        }
        return defaultValue;
    }
}
