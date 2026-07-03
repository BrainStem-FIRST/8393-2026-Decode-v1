package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class TrajectoryLUT {
    public final double distFromGoal;
    public final double relGoalHeight;
    public final double dragCoef;
    public final double magnusCoef;

    private final ArrayList<Trajectory> exitAngTrajectories;
    private final ExitSpeedTrajectories exitSpeedTrajectories;
    public final Trajectory optimalHighArcTrajectory, optimalLowArcTrajectory;

    public TrajectoryLUT(
            double distFromGoal,
            double relGoalHeight,
            double dragCoef,
            double magnusCoef,
            int optimalHighArcIndex,
            int optimalLowArcIndex,
            ArrayList<Trajectory> rawTrajectories) {
        this.distFromGoal = distFromGoal;
        this.relGoalHeight = relGoalHeight;
        this.dragCoef = dragCoef;
        this.magnusCoef = magnusCoef;

        if (rawTrajectories.isEmpty())
            throw new IllegalArgumentException("trajectories must not be empty");
        if (optimalHighArcIndex < 0 || optimalHighArcIndex >= rawTrajectories.size())
            throw new IllegalArgumentException("JSON file has invalid optimal high arc index at " + distFromGoal + "meters. " + optimalHighArcIndex + " must be >= 0 and < " + rawTrajectories.size());
        if (optimalLowArcIndex < 0 || optimalLowArcIndex >= rawTrajectories.size())
            throw new IllegalArgumentException("JSON file has invalid optimal low arc index at " + distFromGoal + "meters. " + optimalLowArcIndex + " must be >= 0 and < " + rawTrajectories.size());

        ArrayList<Trajectory> rawExitAngleSortedTrajectories = new ArrayList<>(rawTrajectories);
        rawExitAngleSortedTrajectories.sort(Comparator.comparingDouble(t -> t.exitAngle.radians()));

        int lowestExitSpeedIndex = -1;
        double lowestExitSpeedMps = -1;

        for (int i=0; i<rawExitAngleSortedTrajectories.size(); i++) {
            Trajectory traj = rawExitAngleSortedTrajectories.get(i);
            if (lowestExitSpeedMps == -1 || traj.exitSpeedMps < lowestExitSpeedMps) {
                lowestExitSpeedMps = traj.exitSpeedMps;
                lowestExitSpeedIndex = i;
            }
        }

        // both of these include the lowest exit speed trajectory
        List<Trajectory> lowArcTrajectories = new ArrayList<>(rawExitAngleSortedTrajectories).subList(0, lowestExitSpeedIndex + 1);
        List<Trajectory> highArcTrajectories = new ArrayList<>(rawExitAngleSortedTrajectories).subList(lowestExitSpeedIndex, rawExitAngleSortedTrajectories.size());

        this.exitAngTrajectories = new ArrayList<>(rawExitAngleSortedTrajectories);
        this.exitSpeedTrajectories = new ExitSpeedTrajectories(lowArcTrajectories, highArcTrajectories);

        optimalHighArcTrajectory = rawTrajectories.get(optimalHighArcIndex);

        optimalLowArcTrajectory = rawTrajectories.get(optimalLowArcIndex);
    }

    public Trajectory getInterpolatedExitSpeedTrajectory(double exitSpeedMps, Angle2d exitAngle) {
        return getInterpolatedExitSpeedTrajectory(exitSpeedMps, isHighArc(exitAngle));
    }
    public Trajectory getInterpolatedExitSpeedTrajectory(double exitSpeedMps, boolean highArc) {
        List<Trajectory> relevantTrajectories = exitSpeedTrajectories.getRelevantTrajectories(highArc);

        if (exitSpeedMps <= relevantTrajectories.get(0).exitSpeedMps)
            return relevantTrajectories.get(0).invalidate();
        if (exitSpeedMps >= relevantTrajectories.get(relevantTrajectories.size() - 1).exitSpeedMps)
            return relevantTrajectories.get(relevantTrajectories.size() - 1).invalidate();

        for (int i = 0; i < relevantTrajectories.size() - 1; i++) {
            Trajectory lo = relevantTrajectories.get(i);
            Trajectory hi = relevantTrajectories.get(i + 1);

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
        throw new IllegalStateException("this should never run in trajectoryLUT.getInterpolatedExitSpeedTrajectory() | exitSpeedMPS: " + exitSpeedMps + " | highArc: " + highArc + " | exitSpeedTrajectories: " + exitSpeedTrajectories);
    }

    public Trajectory getInterpolatedExitAngleTrajectory(Angle2d exitAngle) {
        if (exitAngle.radians() <= exitAngTrajectories.get(0).exitAngle.radians())
            return exitAngTrajectories.get(0);
        if (exitAngle.radians() >= exitAngTrajectories.get(exitAngTrajectories.size() - 1).exitAngle.radians())
            return exitAngTrajectories.get(exitAngTrajectories.size() - 1);


        for (int i = 0; i < exitAngTrajectories.size() - 1; i++) {
            Trajectory lo = exitAngTrajectories.get(i);
            Trajectory hi = exitAngTrajectories.get(i + 1);


            if (exitAngle.radians() >= lo.exitAngle.radians() && exitAngle.radians() <= hi.exitAngle.radians()) {
                double diff = hi.exitAngle.radians() - lo.exitAngle.radians();
                if (diff < 1e-3)
                    return lo;
                double t = (exitAngle.radians() - lo.exitAngle.radians()) / diff;
                return lo.lerp(hi, t);
            }
        }
        throw new IllegalStateException("this should never run in trajectoryLUT.getInterpolatedExitAngleTrajectory() | exitAngle: " + exitAngle + " | exitAngTrajectories: " + exitAngTrajectories);
    }

    public boolean exitSpeedInRange(double exitSpeedMps, Angle2d exitAngle) {
        return exitSpeedMps >= getMinExitSpeedMps() && exitSpeedMps <= getMaxExitSpeedMps(exitAngle);
    }
    public boolean exitAngleInRange(Angle2d exitAngle) {
        return exitAngle.radians() >= getMinExitAngle().radians() && exitAngle.radians() <= getMaxExitAngle().radians();
    }

    public int getNumTrajectories() {
        return exitSpeedTrajectories.getTotalTrajectories();
    }
    public double getMinExitSpeedMps() {
        return exitSpeedTrajectories.getLowestSpeedTrajectory().exitSpeedMps;
    }
    public double getMaxExitSpeedMps(Angle2d exitAngle) {
    return exitSpeedTrajectories.getHighestSpeedTrajectory(isHighArc(exitAngle)).exitSpeedMps;
    }
    public Angle2d getMinExitAngle() {
        return exitAngTrajectories.get(0).exitAngle;
    }
    public Angle2d getMaxExitAngle() {
        return exitAngTrajectories.get(exitAngTrajectories.size() - 1).exitAngle;
    }

    public boolean isHighArc(Angle2d exitAngle) {
        return exitAngle.radians() > exitSpeedTrajectories.getLowestSpeedTrajectory().exitAngle.radians();
    }

    @NonNull
    @Override
    public String toString() {
        DecimalFormat df = new DecimalFormat("0.000");
        return "metersFromGoal: " + df.format(distFromGoal) + " | NumTrajs: " + getNumTrajectories();
    }
    public String getSpeedSortedTrajectoriesString() {
        StringBuilder str = new StringBuilder();
        str.append("LowArc: ");
        for (Trajectory traj : exitSpeedTrajectories.lowArcTrajectories())
            str.append(traj.toStringShort()).append(" | ");
        str.append("HighArc: ");
        for (Trajectory traj : exitSpeedTrajectories.highArcTrajectories())
            str.append(traj.toStringShort()).append(" | ");
        return str.toString().substring(0, str.length() - 3);
    }
}
