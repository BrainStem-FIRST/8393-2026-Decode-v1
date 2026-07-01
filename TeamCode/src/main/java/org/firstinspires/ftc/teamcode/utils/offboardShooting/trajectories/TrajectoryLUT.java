package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import java.util.ArrayList;
import java.util.Comparator;

public class TrajectoryLUT {
    public final double distFromGoal;
    public final double relGoalHeight;
    public final double dragCoef;
    public final double magnusCoef;

    private final ArrayList<Trajectory> impactAngSortedTrajectories;
    private final ArrayList<Trajectory> exitAngSortedTrajectories;
    private final ArrayList<Trajectory> speedSortedTrajectories;
    private final Trajectory optimalTrajectory;

    public TrajectoryLUT(
            double distFromGoal,
            double relGoalHeight,
            double dragCoef,
            double magnusCoef,
            int optimalTrajectoryIndex,
            ArrayList<Trajectory> trajectories) {
        this.distFromGoal = distFromGoal;
        this.relGoalHeight = relGoalHeight;
        this.dragCoef = dragCoef;
        this.magnusCoef = magnusCoef;

        if (trajectories.isEmpty())
            throw new IllegalArgumentException("trajectories must not be empty");
        if (optimalTrajectoryIndex < 0 || optimalTrajectoryIndex >= trajectories.size())
            throw new IllegalArgumentException("optimalTrajectoryIndex out of range");

        this.impactAngSortedTrajectories = new ArrayList<>(trajectories);
        this.exitAngSortedTrajectories = new ArrayList<>(trajectories);
        this.speedSortedTrajectories = new ArrayList<>(trajectories);

        this.impactAngSortedTrajectories.sort(Comparator.comparingDouble(t -> t.impactAngleRad));
        this.exitAngSortedTrajectories.sort(Comparator.comparingDouble(t -> t.exitAngleRad));
        this.speedSortedTrajectories.sort(Comparator.comparingDouble(t -> t.exitSpeedMps));

        this.optimalTrajectory = trajectories.get(optimalTrajectoryIndex);
    }

    public Trajectory getOptimalTrajectory() {
        return optimalTrajectory;
    }

    public Trajectory getInterpolatedImpactAngleTrajectory(double impactAngleRad) {
        if (impactAngSortedTrajectories.isEmpty())
            return null;

        if (impactAngleRad <= impactAngSortedTrajectories.get(0).impactAngleRad)
            return impactAngSortedTrajectories.get(0);

        if (impactAngleRad >= impactAngSortedTrajectories.get(impactAngSortedTrajectories.size() - 1).impactAngleRad)
            return impactAngSortedTrajectories.get(impactAngSortedTrajectories.size() - 1);

        for (int i = 0; i < impactAngSortedTrajectories.size() - 1; i++) {
            Trajectory lo = impactAngSortedTrajectories.get(i);
            Trajectory hi = impactAngSortedTrajectories.get(i + 1);

            double loRad = lo.impactAngleRad;
            double hiRad = hi.impactAngleRad;

            if (impactAngleRad >= loRad && impactAngleRad <= hiRad) {
                double diff = hiRad - loRad;
                if (diff < 1e-3)
                    return lo;
                double t = (impactAngleRad - loRad) / diff;
                return lo.lerp(hi, t);
            }
        }
        return null;
    }

    public Trajectory getInterpolatedExitSpeedTrajectory(double exitSpeedMps) {
        if (speedSortedTrajectories.isEmpty())
            return null;

        if (exitSpeedMps <= speedSortedTrajectories.get(0).exitSpeedMps)
            return speedSortedTrajectories.get(0);
        if (exitSpeedMps >= speedSortedTrajectories.get(speedSortedTrajectories.size() - 1).exitSpeedMps)
            return speedSortedTrajectories.get(speedSortedTrajectories.size() - 1);

        for (int i = 0; i < speedSortedTrajectories.size() - 1; i++) {
            Trajectory lo = speedSortedTrajectories.get(i);
            Trajectory hi = speedSortedTrajectories.get(i + 1);

            double loSpeed = lo.exitSpeedMps;
            double hiSpeed = hi.exitSpeedMps;

            if (exitSpeedMps >= loSpeed && exitSpeedMps <= hiSpeed) {
                double diff = hiSpeed - loSpeed;
                if (diff < 1e-3)
                    return lo;
                double t = (exitSpeedMps - loSpeed) / diff;
                return lo.lerp(hi, t);
            }
        }
        return null;
    }

    public Trajectory getInterpolatedExitAngleTrajectory(double exitAngleRad) {
        if (exitAngSortedTrajectories.isEmpty())
            return null;

        if (exitAngleRad <= exitAngSortedTrajectories.get(0).exitAngleRad)
            return exitAngSortedTrajectories.get(0);
        if (exitAngleRad >= exitAngSortedTrajectories.get(exitAngSortedTrajectories.size() - 1).exitAngleRad)
            return exitAngSortedTrajectories.get(exitAngSortedTrajectories.size() - 1);

        for (int i = 0; i < exitAngSortedTrajectories.size() - 1; i++) {
            Trajectory lo = exitAngSortedTrajectories.get(i);
            Trajectory hi = exitAngSortedTrajectories.get(i + 1);

            double loAng = lo.exitAngleRad;
            double hiAng = hi.exitAngleRad;

            if (exitAngleRad >= loAng && exitAngleRad <= hiAng) {
                double diff = hiAng - loAng;
                if (diff < 1e-3)
                    return lo;
                double t = (exitAngleRad - loAng) / diff;
                return lo.lerp(hi, t);
            }
        }
        return null;
    }

    public boolean exitSpeedInRange(double exitSpeedMps) {
        if (speedSortedTrajectories.isEmpty())
            return false;
        return exitSpeedMps >= getMinExitSpeedMps() && exitSpeedMps <= getMaxExitSpeedMps();
    }
    public boolean exitAngleInRange(double exitAngleRad) {
        if (exitAngSortedTrajectories.isEmpty())
            return false;
        return exitAngleRad >= getMinExitAngleRad() && exitAngleRad <= getMaxExitAngleRad();
    }
    public boolean impactAngleInRange(double impactAngleRad) {
        if (impactAngSortedTrajectories.isEmpty())
            return false;
        return impactAngleRad >= getMinImpactAngleRad() && impactAngleRad <= getMaxImpactAngleRad();
    }

    public int getNumTrajectories() {
        return speedSortedTrajectories.size();
    }
    public double getMinExitSpeedMps() {
        return speedSortedTrajectories.get(0).exitSpeedMps;
    }
    public double getMaxExitSpeedMps() {
        return speedSortedTrajectories.get(speedSortedTrajectories.size() - 1).exitSpeedMps;
    }
    public double getMinExitAngleRad() {
        return exitAngSortedTrajectories.get(0).exitAngleRad;
    }
    public double getMaxExitAngleRad() {
        return exitAngSortedTrajectories.get(exitAngSortedTrajectories.size() - 1).exitAngleRad;
    }
    public double getMinImpactAngleRad() {
        return impactAngSortedTrajectories.get(0).impactAngleRad;
    }
    public double getMaxImpactAngleRad() {
        return impactAngSortedTrajectories.get(impactAngSortedTrajectories.size() - 1).impactAngleRad;
    }
}
