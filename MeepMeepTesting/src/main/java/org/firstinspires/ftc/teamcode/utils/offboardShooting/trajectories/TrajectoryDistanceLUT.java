package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;

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

    public TrajectoryWrapper getInterpolatedExitSpeedTrajectory(double distFromGoal, double exitSpeed, Angle2d targetExitAngle) {
        if (distFromGoal <= getMinDistance())
            return trajectoryLUTs.get(0).getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngle).invalidate();
        if (distFromGoal >= getMaxDistance())
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngle).invalidate();

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngle);

        double t = (distFromGoal - neighbors.loDist) / distRange;
        TrajectoryWrapper loTraj = neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngle);
        TrajectoryWrapper hiTraj = neighbors.hiLUT.getInterpolatedExitSpeedTrajectory(exitSpeed, targetExitAngle);

        return loTraj.lerp(hiTraj, t);
    }

    public TrajectoryWrapper getInterpolatedExitAngleTrajectory(double distFromGoal, Angle2d exitAngle) {
        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getInterpolatedExitAngleTrajectory(exitAngle).invalidate();
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedExitAngleTrajectory(exitAngle).invalidate();

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedExitAngleTrajectory(exitAngle);

        double t = (distFromGoal - neighbors.loDist) / distRange;

        TrajectoryWrapper loTraj = neighbors.loLUT.getInterpolatedExitAngleTrajectory(exitAngle);
        TrajectoryWrapper hiTraj = neighbors.hiLUT.getInterpolatedExitAngleTrajectory(exitAngle);

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

        trajectoryLUTs.sort(Comparator.comparingDouble(t -> t.distFromGoal));

        for (int i = 0; i < trajectoryLUTs.size() - 1; i++) {
            TrajectoryLUT loLUT = trajectoryLUTs.get(i);
            TrajectoryLUT hiLUT = trajectoryLUTs.get(i + 1);
            if (loLUT.distFromGoal == hiLUT.distFromGoal)
                throw new IllegalArgumentException("trajectoryLUTs must have unique distFromGoal values");
        }

        TrajectoryDistanceLUT lut = new TrajectoryDistanceLUT();
        lut.trajectoryLUTs.addAll(trajectoryLUTs);
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
    public boolean exitSpeedInRange(double distFromGoal, double exitSpeedMps, Angle2d targetExitAngle) {
        return exitSpeedMps >= getMinExitSpeedMps(distFromGoal) && exitSpeedMps <= getMaxExitSpeedMps(distFromGoal, targetExitAngle);
    }
    public boolean exitAngleInRange(double distFromGoal, Angle2d exitAngle) {
        return exitAngle.radians() >= getMinExitAngle(distFromGoal).radians() && exitAngle.radians() <= getMaxExitAngle(distFromGoal).radians();
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

    public double getMaxExitSpeedMps(double distFromGoal, Angle2d targetExitAngle) {
        DistanceContext ctx = resolveDistanceContext(distFromGoal);
        if (ctx == null)
            return 0;
        if (ctx.isSingle())
            return ctx.loLUT.getMaxExitSpeedMps(targetExitAngle);
        return lerp(
                ctx.loLUT.getMaxExitSpeedMps(targetExitAngle),
                ctx.hiLUT.getMaxExitSpeedMps(targetExitAngle),
                ctx.blendT);
    }

    public Angle2d getMinExitAngle(double distFromGoal) {
        DistanceContext ctx = resolveDistanceContext(distFromGoal);
        if (ctx == null)
            return Angle2d.fromDegrees(0);
        if (ctx.isSingle())
            return ctx.loLUT.getMinExitAngle();
        return ctx.loLUT.getMinExitAngle().lerp(ctx.hiLUT.getMinExitAngle(), ctx.blendT);
    }

    public Angle2d getMaxExitAngle(double distFromGoal) {
        DistanceContext ctx = resolveDistanceContext(distFromGoal);
        if (ctx == null)
            return Angle2d.fromDegrees(0);
        if (ctx.isSingle())
            return ctx.loLUT.getMaxExitAngle();
        return ctx.loLUT.getMaxExitAngle().lerp(ctx.hiLUT.getMaxExitAngle(), ctx.blendT);
    }
}
