package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import java.util.ArrayList;
import java.util.Comparator;

public class TrajectoryDistanceLUT {
    private final ArrayList<TrajectoryLUT> trajectoryLUTs;

    private TrajectoryDistanceLUT() {
        this.trajectoryLUTs = new ArrayList<>();
    }

    public Trajectory getInterpolatedOptimalTrajectory(double distFromGoal) {
        if (trajectoryLUTs.isEmpty())
            return null;

        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getOptimalTrajectory();
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getOptimalTrajectory();

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return null;

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getOptimalTrajectory();

        double t = (distFromGoal - neighbors.loDist) / distRange;

        Trajectory loTraj = neighbors.loLUT.getOptimalTrajectory();
        Trajectory hiTraj = neighbors.hiLUT.getOptimalTrajectory();

        if (loTraj == null || hiTraj == null)
            return null;

        return loTraj.lerp(hiTraj, t);
    }


    public Trajectory getInterpolatedImpactAngleTrajectory(double distFromGoal, double impactAngleRad) {
        if (trajectoryLUTs.isEmpty())
            return null;

        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getInterpolatedImpactAngleTrajectory(impactAngleRad);
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedImpactAngleTrajectory(impactAngleRad);

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return null;

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedImpactAngleTrajectory(impactAngleRad);

        double t = (distFromGoal - neighbors.loDist) / distRange;

        Trajectory loTraj = neighbors.loLUT.getInterpolatedImpactAngleTrajectory(impactAngleRad);
        Trajectory hiTraj = neighbors.hiLUT.getInterpolatedImpactAngleTrajectory(impactAngleRad);

        if (loTraj == null || hiTraj == null)
            return null;

        return loTraj.lerp(hiTraj, t);
    }

    public Trajectory getInterpolatedExitSpeedTrajectory(double distFromGoal, double exitSpeed) {
        if (trajectoryLUTs.isEmpty())
            return null;

        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getInterpolatedExitSpeedTrajectory(exitSpeed);
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedExitSpeedTrajectory(exitSpeed);

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return null;

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed);

        double t = (distFromGoal - neighbors.loDist) / distRange;

        Trajectory loTraj = neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed);
        Trajectory hiTraj = neighbors.hiLUT.getInterpolatedExitSpeedTrajectory(exitSpeed);

        if (loTraj == null || hiTraj == null)
            return null;

        return loTraj.lerp(hiTraj, t);
    }

    public Trajectory getInterpolatedExitAngleTrajectory(double distFromGoal, double exitAngleRad) {
        if (trajectoryLUTs.isEmpty())
            return null;

        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getInterpolatedExitAngleTrajectory(exitAngleRad);
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedExitAngleTrajectory(exitAngleRad);

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return null;

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedExitAngleTrajectory(exitAngleRad);

        double t = (distFromGoal - neighbors.loDist) / distRange;

        Trajectory loTraj = neighbors.loLUT.getInterpolatedExitAngleTrajectory(exitAngleRad);
        Trajectory hiTraj = neighbors.hiLUT.getInterpolatedExitAngleTrajectory(exitAngleRad);

        if (loTraj == null || hiTraj == null)
            return null;

        return loTraj.lerp(hiTraj, t);
    }

    private NeighborTrajectoryInfo getNeighboringTrajectoryLUTs(double distFromGoal) {
        if (!distanceInRange(distFromGoal) || trajectoryLUTs.isEmpty())
            return null;

        for (int i = 0; i < trajectoryLUTs.size() - 1; i++) {
            TrajectoryLUT loLUT = trajectoryLUTs.get(i);
            TrajectoryLUT hiLUT = trajectoryLUTs.get(i + 1);

            double loDist = loLUT.distFromGoal;
            double hiDist = hiLUT.distFromGoal;

            if (distFromGoal >= loDist && distFromGoal <= hiDist)
                return new NeighborTrajectoryInfo(loLUT, hiLUT, loDist, hiDist);
        }
        return null;
    }

    private record NeighborTrajectoryInfo(TrajectoryLUT loLUT, TrajectoryLUT hiLUT, double loDist, double hiDist) {}

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

        return distFromGoal >= trajectoryLUTs.get(0).distFromGoal
                && distFromGoal <= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal;
    }



    public boolean exitSpeedInRange(double distFromGoal, double exitSpeedMps) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return false;
        return neighbors.loLUT.exitSpeedInRange(exitSpeedMps)
                && neighbors.hiLUT.exitSpeedInRange(exitSpeedMps);
    }
    public boolean exitAngleInRange(double distFromGoal, double exitAngleRad) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return false;
        return neighbors.loLUT.exitAngleInRange(exitAngleRad)
                && neighbors.hiLUT.exitAngleInRange(exitAngleRad);
    }
    public boolean impactAngleInRange(double distFromGoal, double impactAngleRad) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return false;
        return neighbors.loLUT.impactAngleInRange(impactAngleRad)
                && neighbors.hiLUT.impactAngleInRange(impactAngleRad);
    }

    public double getMinExitSpeedMps(double distFromGoal) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return 0;
        return Math.max(neighbors.loLUT.getMinExitSpeedMps(), neighbors.hiLUT.getMinExitSpeedMps());
    }
    public double getMaxExitSpeedMps(double distFromGoal) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return 0;
        return Math.min(neighbors.loLUT.getMaxExitSpeedMps(), neighbors.hiLUT.getMaxExitSpeedMps());
    }
    public double getMinExitAngleRad(double distFromGoal) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return 0;
        return Math.max(neighbors.loLUT.getMinExitAngleRad(), neighbors.hiLUT.getMinExitAngleRad());
    }
    public double getMaxExitAngleRad(double distFromGoal) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return 0;
        return Math.min(neighbors.loLUT.getMaxExitAngleRad(), neighbors.hiLUT.getMaxExitAngleRad());
    }
    public double getMinImpactAngleRad(double distFromGoal) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return 0;
        return Math.max(neighbors.loLUT.getMinImpactAngleRad(), neighbors.hiLUT.getMinImpactAngleRad());
    }
    public double getMaxImpactAngleRad(double distFromGoal) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return 0;
        return Math.min(neighbors.loLUT.getMaxImpactAngleRad(), neighbors.hiLUT.getMaxImpactAngleRad());
    }
}
