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
    private final TrajectoryWrapper optimalTrajectory;

    public TrajectoryLUT(
            double distFromGoal,
            double relGoalHeight,
            double dragCoef,
            double magnusCoef,
            int optimalTrajectoryIndex,
            ArrayList<Trajectory> rawTrajectories) {
        this.distFromGoal = distFromGoal;
        this.relGoalHeight = relGoalHeight;
        this.dragCoef = dragCoef;
        this.magnusCoef = magnusCoef;

        if (rawTrajectories.isEmpty())
            throw new IllegalArgumentException("trajectories must not be empty");
        if (optimalTrajectoryIndex < 0 || optimalTrajectoryIndex >= rawTrajectories.size())
            throw new IllegalArgumentException("optimalTrajectoryIndex out of range");

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

        Trajectory optimalTrajectory = rawTrajectories.get(optimalTrajectoryIndex); // optimal trajectory is not same as lowest speed trajectory
        TrajectoryType optimalTrajectoryType = exitSpeedTrajectories.getTrajectoryType(optimalTrajectory.exitAngle, rawExitAngleSortedTrajectories.size());
        this.optimalTrajectory = new TrajectoryWrapper(optimalTrajectory, optimalTrajectoryType);
    }

    public TrajectoryWrapper getOptimalTrajectory() {
        return optimalTrajectory;
    }

    public TrajectoryWrapper getInterpolatedExitSpeedTrajectory(double exitSpeedMps, Angle2d targetExitAngle) {
        List<Trajectory> relevantTrajectories = exitSpeedTrajectories.getRelevantTrajectories(targetExitAngle);
        TrajectoryType trajectoryType = exitSpeedTrajectories.getTrajectoryType(targetExitAngle, exitSpeedTrajectories.getTotalTrajectories());

        if (exitSpeedMps <= relevantTrajectories.get(0).exitSpeedMps)
            return new TrajectoryWrapper(relevantTrajectories.get(0).invalidate(), TrajectoryType.LOWEST_SPEED);
        if (exitSpeedMps >= relevantTrajectories.get(relevantTrajectories.size() - 1).exitSpeedMps)
            return new TrajectoryWrapper(relevantTrajectories.get(relevantTrajectories.size() - 1).invalidate(), trajectoryType);

        for (int i = 0; i < relevantTrajectories.size() - 1; i++) {
            Trajectory lo = relevantTrajectories.get(i);
            Trajectory hi = relevantTrajectories.get(i + 1);

            double loSpeed = lo.exitSpeedMps;
            double hiSpeed = hi.exitSpeedMps;

            if (exitSpeedMps >= loSpeed && exitSpeedMps <= hiSpeed) {
                double diff = hiSpeed - loSpeed;
                if (diff < 1e-3)
                    return new TrajectoryWrapper(lo, trajectoryType);
                double t = (exitSpeedMps - loSpeed) / diff;
                return new TrajectoryWrapper(lo.lerp(hi, t), trajectoryType);
            }
        }
        throw new IllegalStateException("this should never run in trajectoryLUT.getInterpolatedExitSpeedTrajectory() | exitSpeedMPS: " + exitSpeedMps + " | targetExitAngleDeg: " + targetExitAngle.degrees() + " | exitSpeedTrajectories: " + exitSpeedTrajectories);
    }

    public TrajectoryWrapper getInterpolatedExitAngleTrajectory(Angle2d exitAngle) {
        if (exitAngle.radians() <= exitAngTrajectories.get(0).exitAngle.radians())
            return new TrajectoryWrapper(exitAngTrajectories.get(0), TrajectoryType.LOW_ARC);
        if (exitAngle.radians() >= exitAngTrajectories.get(exitAngTrajectories.size() - 1).exitAngle.radians())
            return new TrajectoryWrapper(exitAngTrajectories.get(exitAngTrajectories.size() - 1), TrajectoryType.HIGH_ARC);


        for (int i = 0; i < exitAngTrajectories.size() - 1; i++) {
            Trajectory lo = exitAngTrajectories.get(i);
            Trajectory hi = exitAngTrajectories.get(i + 1);

            TrajectoryType loType = exitSpeedTrajectories.getTrajectoryType(lo.exitAngle, exitSpeedTrajectories.getTotalTrajectories());
            TrajectoryType hiType = exitSpeedTrajectories.getTrajectoryType(hi.exitAngle, exitSpeedTrajectories.getTotalTrajectories());

            if (exitAngle.radians() >= lo.exitAngle.radians() && exitAngle.radians() <= hi.exitAngle.radians()) {
                double diff = hi.exitAngle.radians() - lo.exitAngle.radians();
                if (diff < 1e-3)
                    return new TrajectoryWrapper(lo, loType);
                double t = (exitAngle.radians() - lo.exitAngle.radians()) / diff;
                return new TrajectoryWrapper(lo, loType).lerp(new TrajectoryWrapper(hi, hiType), t);
            }
        }
        throw new IllegalStateException("this should never run in trajectoryLUT.getInterpolatedExitAngleTrajectory() | exitAngle: " + exitAngle + " | exitAngTrajectories: " + exitAngTrajectories);
    }

    public boolean exitSpeedInRange(double exitSpeedMps, Angle2d targetExitAngle) {
        return exitSpeedMps >= getMinExitSpeedMps() && exitSpeedMps <= getMaxExitSpeedMps(targetExitAngle);
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
    public double getMaxExitSpeedMps(Angle2d targetExitAngle) {
        return exitSpeedTrajectories.getHighestSpeedTrajectory(targetExitAngle).exitSpeedMps;
    }
    public Angle2d getMinExitAngle() {
        return exitAngTrajectories.get(0).exitAngle;
    }
    public Angle2d getMaxExitAngle() {
        return exitAngTrajectories.get(exitAngTrajectories.size() - 1).exitAngle;
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
