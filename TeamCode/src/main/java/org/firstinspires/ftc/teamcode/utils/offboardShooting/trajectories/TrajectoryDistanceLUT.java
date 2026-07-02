package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import java.util.ArrayList;
import java.util.Comparator;

public class TrajectoryDistanceLUT {
    private final ArrayList<TrajectoryLUT> trajectoryLUTs;

    private TrajectoryDistanceLUT() {
        this.trajectoryLUTs = new ArrayList<>();
    }

    public TrajectoryWrapper getInterpolatedOptimalTrajectory(double distFromGoal) {
        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getOptimalTrajectory().invalidate();
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getOptimalTrajectory().invalidate();

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getOptimalTrajectory();

        double t = (distFromGoal - neighbors.loDist) / distRange;

        TrajectoryWrapper loTraj = neighbors.loLUT.getOptimalTrajectory();
        TrajectoryWrapper hiTraj = neighbors.hiLUT.getOptimalTrajectory();

        return loTraj.lerp(hiTraj, t);
    }

    public TrajectoryWrapper getInterpolatedExitSpeedTrajectory(double distFromGoal, double exitSpeed, double targetExitAngleRad) {
        if (distFromGoal <= getMinDistance())
            return trajectoryLUTs.get(0).getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngleRad).invalidate();
        if (distFromGoal >= getMaxDistance())
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngleRad).invalidate();

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngleRad);

        double t = (distFromGoal - neighbors.loDist) / distRange;
        TrajectoryWrapper loTraj = neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngleRad);
        TrajectoryWrapper hiTraj = neighbors.hiLUT.getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngleRad);

        return loTraj.lerp(hiTraj, t);
    }

    public TrajectoryWrapper getInterpolatedExitAngleTrajectory(double distFromGoal, double exitAngleRad) {
        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getInterpolatedExitAngleTrajectory(exitAngleRad).invalidate();
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedExitAngleTrajectory(exitAngleRad).invalidate();

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedExitAngleTrajectory(exitAngleRad);

        double t = (distFromGoal - neighbors.loDist) / distRange;

        TrajectoryWrapper loTraj = neighbors.loLUT.getInterpolatedExitAngleTrajectory(exitAngleRad);
        TrajectoryWrapper hiTraj = neighbors.hiLUT.getInterpolatedExitAngleTrajectory(exitAngleRad);

        return loTraj.lerp(hiTraj, t);
    }

    public NeighborTrajectoryInfo getNeighboringTrajectoryLUTs(double distFromGoal) {
        if (!distanceInRange(distFromGoal))
            throw new IllegalArgumentException("distance passed to getNeighboringTrajectoryLUTs must be in range. " + distFromGoal + " is not in the range of " + getMinDistance() + "-" + getMaxDistance());

        for (int i = 0; i < trajectoryLUTs.size() - 1; i++) {
            TrajectoryLUT loLUT = trajectoryLUTs.get(i);
            TrajectoryLUT hiLUT = trajectoryLUTs.get(i + 1);

            double loDist = loLUT.distFromGoal;
            double hiDist = hiLUT.distFromGoal;

            if (distFromGoal >= loDist && distFromGoal <= hiDist)
                return new NeighborTrajectoryInfo(loLUT, hiLUT, loDist, hiDist);
        }
        throw new IllegalStateException("this should never happen in getNeighboringTrajectoryLUTs. distFromGoal: " + distFromGoal + " trajectoryLUTs: " + trajectoryLUTs);
    }


    public record NeighborTrajectoryInfo(TrajectoryLUT loLUT, TrajectoryLUT hiLUT, double loDist, double hiDist) {}

    private record DistanceContext(TrajectoryLUT loLUT, TrajectoryLUT hiLUT, double blendT) {
        static DistanceContext single(TrajectoryLUT lut) {
            return new DistanceContext(lut, null, Double.NaN);
        }
        static DistanceContext blend(TrajectoryLUT loLUT, TrajectoryLUT hiLUT, double blendT) {
            return new DistanceContext(loLUT, hiLUT, blendT);
        }
        boolean isSingle() {
            return hiLUT == null;
        }
    }

    /**
     * Resolves which LUT(s) and blend weight apply at a distance, matching the interpolators'
     * clamp and lerp behavior.
     */
    private DistanceContext resolveDistanceContext(double distFromGoal) {
        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return DistanceContext.single(trajectoryLUTs.get(0));
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return DistanceContext.single(trajectoryLUTs.get(trajectoryLUTs.size() - 1));

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return DistanceContext.single(neighbors.loLUT);

        double t = (distFromGoal - neighbors.loDist) / distRange;
        return DistanceContext.blend(neighbors.loLUT, neighbors.hiLUT, t);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static TrajectoryDistanceLUT fromTrajectoryLUTs(ArrayList<TrajectoryLUT> trajectoryLUTs) {
        if (trajectoryLUTs.isEmpty())
            throw new IllegalArgumentException("trajectoryLUTs cannot be empty");
        if (trajectoryLUTs.size() < 2)
            throw new IllegalArgumentException("trajectoryLUTs must contain at least 2 elements");
        for (int i = 0; i < trajectoryLUTs.size() - 1; i++) {
            TrajectoryLUT loLUT = trajectoryLUTs.get(i);
            TrajectoryLUT hiLUT = trajectoryLUTs.get(i + 1);
            if (loLUT.distFromGoal >= hiLUT.distFromGoal)
                throw new IllegalArgumentException("trajectoryLUTs must be sorted by distFromGoal");
        }

        TrajectoryDistanceLUT lut = new TrajectoryDistanceLUT();
        lut.trajectoryLUTs.addAll(trajectoryLUTs);
        lut.trajectoryLUTs.sort(Comparator.comparingDouble(t -> t.distFromGoal));
        return lut;
    }

    public double getRelGoalHeight() {
        if (trajectoryLUTs.isEmpty())
            return 0;
        return trajectoryLUTs.get(0).relGoalHeight;
    }
    public boolean distanceInRange(double distFromGoal) {
        if (trajectoryLUTs.isEmpty())
            return false;

        return distFromGoal >= getMinDistance() && distFromGoal <= getMaxDistance();
    }
    public boolean exitSpeedInRange(double distFromGoal, double exitSpeedMps, double targetExitAngleRad) {
        return exitSpeedMps >= getMinExitSpeedMps(distFromGoal) && exitSpeedMps <= getMaxExitSpeedMps(distFromGoal, targetExitAngleRad);
    }
    public boolean exitAngleInRange(double distFromGoal, double exitAngleRad) {
        return exitAngleRad >= getMinExitAngleRad(distFromGoal) && exitAngleRad <= getMaxExitAngleRad(distFromGoal);
    }

    public double getMinDistance() {
        return trajectoryLUTs.get(0).distFromGoal;
    }
    public double getMaxDistance() {
        return trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal;
    }
    public double getMinExitSpeedMps(double distFromGoal) {
        DistanceContext ctx = resolveDistanceContext(distFromGoal);
        if (ctx == null)
            return 0;
        if (ctx.isSingle())
            return ctx.loLUT.getMinExitSpeedMps();
        return lerp(ctx.loLUT.getMinExitSpeedMps(), ctx.hiLUT.getMinExitSpeedMps(), ctx.blendT);
    }

    public double getMaxExitSpeedMps(double distFromGoal, double targetExitAngleRad) {
        DistanceContext ctx = resolveDistanceContext(distFromGoal);
        if (ctx == null)
            return 0;
        if (ctx.isSingle())
            return ctx.loLUT.getMaxExitSpeedMps(targetExitAngleRad);
        return lerp(
                ctx.loLUT.getMaxExitSpeedMps(targetExitAngleRad),
                ctx.hiLUT.getMaxExitSpeedMps(targetExitAngleRad),
                ctx.blendT);
    }

    public double getMinExitAngleRad(double distFromGoal) {
        DistanceContext ctx = resolveDistanceContext(distFromGoal);
        if (ctx == null)
            return 0;
        if (ctx.isSingle())
            return ctx.loLUT.getMinExitAngleRad();
        return lerp(ctx.loLUT.getMinExitAngleRad(), ctx.hiLUT.getMinExitAngleRad(), ctx.blendT);
    }

    public double getMaxExitAngleRad(double distFromGoal) {
        DistanceContext ctx = resolveDistanceContext(distFromGoal);
        if (ctx == null)
            return 0;
        if (ctx.isSingle())
            return ctx.loLUT.getMaxExitAngleRad();
        return lerp(ctx.loLUT.getMaxExitAngleRad(), ctx.hiLUT.getMaxExitAngleRad(), ctx.blendT);
    }
}
