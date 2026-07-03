package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;

import java.util.Comparator;
import java.util.List;

public record ExitSpeedTrajectories(List<Trajectory> lowArcTrajectories, List<Trajectory> highArcTrajectories) {
    // both lowArcTrajectories and highArcTrajectories should include trajectory with lowest speed
    public ExitSpeedTrajectories(List<Trajectory> lowArcTrajectories, List<Trajectory> highArcTrajectories) {
        if (lowArcTrajectories.isEmpty() && highArcTrajectories.isEmpty())
            throw new IllegalArgumentException("both lowArcTrajectories and highArcTrajectories are empty when instantiating ExitSpeedTrajectories");
        if (lowArcTrajectories.isEmpty())
            throw new IllegalArgumentException("lowArcTrajectories is empty when instantiating ExitSpeedTrajectories. highArcTrajectories is not empty: " + highArcTrajectories);
        if (highArcTrajectories.isEmpty())
            throw new IllegalArgumentException("highArcTrajectories is empty when instantiating ExitSpeedTrajectories. lowArcTrajectories is not empty: " + lowArcTrajectories);

        this.lowArcTrajectories = lowArcTrajectories;
        this.highArcTrajectories = highArcTrajectories;

        this.lowArcTrajectories.sort(Comparator.comparingDouble(traj -> traj.exitSpeedMps));
        this.highArcTrajectories.sort(Comparator.comparingDouble(traj -> traj.exitSpeedMps));
        Trajectory lowArcLowest = this.lowArcTrajectories.get(0);
        Trajectory highArcLowest = this.highArcTrajectories.get(0);
        if (!lowArcLowest.exitAngle.epsilonEquals(highArcLowest.exitAngle, 0.001))
            throw new IllegalArgumentException("lowArcTrajectories and highArcTrajectories should both contain lowest speed trajectory. LowArcLowest: " + lowArcLowest + " | HighArcLowest: " + highArcLowest);
    }

    public int getTotalTrajectories() {
        return lowArcTrajectories.size() + highArcTrajectories.size() - 1; // minus one to account for duplicate lowest speed trajectory
    }

    public Trajectory getLowestSpeedTrajectory() {
        return highArcTrajectories.get(0);
    }

    public Trajectory getHighestSpeedTrajectory(boolean highArc) {
        List<Trajectory> trajectories = getRelevantTrajectories(highArc);
        return trajectories.get(trajectories.size() - 1);
    }

    public List<Trajectory> getRelevantTrajectories(boolean highArc) {
        return highArc ? highArcTrajectories : lowArcTrajectories;
    }
}
