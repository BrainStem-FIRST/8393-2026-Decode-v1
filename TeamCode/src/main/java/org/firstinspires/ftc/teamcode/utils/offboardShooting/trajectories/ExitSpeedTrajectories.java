package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import java.util.Comparator;
import java.util.List;

public class ExitSpeedTrajectories {
    public final List<Trajectory> lowArcTrajectories, highArcTrajectories;
    // both lowArcTrajectories and highArcTrajectories should include trajectory with lowest speed
    public ExitSpeedTrajectories(List<Trajectory> lowArcTrajectories, List<Trajectory> highArcTrajectories) {
        this.lowArcTrajectories = lowArcTrajectories;
        this.highArcTrajectories = highArcTrajectories;

        this.lowArcTrajectories.sort(Comparator.comparingDouble(traj -> traj.exitSpeedMps));
        this.highArcTrajectories.sort(Comparator.comparingDouble(traj -> traj.exitSpeedMps));
    }

    public int getTotalTrajectories() {
        return lowArcTrajectories.size() + highArcTrajectories.size() - 1; // minus one to account for duplicate lowest speed trajectory
    }
    public Trajectory getLowestSpeedTrajectory() {
        return highArcTrajectories.get(0);
    }
    public Trajectory getHighestSpeedTrajectory(double targetExitAngleRad) {
        List<Trajectory> trajectories = getRelevantTrajectories(targetExitAngleRad);
        return trajectories.get(trajectories.size() - 1);
    }

    // assumes totalTrajectories() returns more than one
    public TrajectoryType getTrajectoryType(double exitAngleRad) {
        if (exitAngleRad > getLowestSpeedTrajectory().exitAngleRad)
            return TrajectoryType.HIGH_ARC;
        if (exitAngleRad == getLowestSpeedTrajectory().exitAngleRad)
            return TrajectoryType.LOWEST_SPEED;
        return TrajectoryType.LOW_ARC;
    }
    public List<Trajectory> getRelevantTrajectories(double exitAngleRad) {
        if (exitAngleRad > getLowestSpeedTrajectory().exitAngleRad)
            return highArcTrajectories;
        return lowArcTrajectories;
    }
}
