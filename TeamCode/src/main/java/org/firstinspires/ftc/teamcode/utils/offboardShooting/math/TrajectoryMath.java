package org.firstinspires.ftc.teamcode.utils.offboardShooting.math;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryDistanceLUT;

/**
 * Utility methods for applying robot-motion compensation to offboard-generated
 * shooting trajectories.
 *
 * <p>The trajectory database assumes a stationary robot. This class estimates
 * the apparent displacement of the goal caused by robot and turret motion
 * during the projectile's time of flight, allowing the turret to "lead" the
 * target.
 */
public class TrajectoryMath {
    public record TargetingInfo(
        Trajectory targetTrajectory,
        Trajectory compensatedTrajectory,
        Vec2d displacedGoal,
        Angle2d turretFieldAngle
    ) {}

    /**
     * Computes the displaced goal and the last trajectory after several iterations,
     * updating based on the provided function for next trajectory lookup.
     * turret angle is always based on targetTrajectory, so this assumes change in ToF between target and compensated is negligible
     */
    private static TrajectoryGoalInfo computeDisplacedGoal(
        TrajectoryDistanceLUT trajectoryLUT,
        Vec2d goalPos,
        Vec2d turretPos,
        Vec2d turretVel,
        double startDistFromGoalM,
        double curExitSpeedMps,
        int tofEstimationIterations,
        boolean highArc
    ) {
        Vec2d displacedGoal = goalPos;
        Vec2d turretToGoal;
        double distFromGoal = startDistFromGoalM;
        Trajectory targetTrajectory = trajectoryLUT.getInterpolatedOptimalTrajectory(distFromGoal, highArc);

        for (int i = 0; i < tofEstimationIterations; i++) {
            Vec2d displacement = turretVel.times(-1).times(targetTrajectory.timeOfFlight * 0.85);
            displacedGoal = goalPos.plus(displacement);
            turretToGoal = displacedGoal.minus(turretPos);
            distFromGoal = turretToGoal.norm();

            targetTrajectory = trajectoryLUT.getInterpolatedOptimalTrajectory(distFromGoal, highArc);
        }
        Trajectory compensatedTrajectory = trajectoryLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, curExitSpeedMps, highArc);
        return new TrajectoryGoalInfo(targetTrajectory, compensatedTrajectory, displacedGoal, distFromGoal);
    }

    private record TrajectoryGoalInfo(
            Trajectory targetTrajectory,
            Trajectory compensatedTrajectory,
            Vec2d displacedGoal,
            double distFromGoal) {}

    // TAKES EVERYTHING IN METERS
    public static TargetingInfo calculateTargetingInfo(
        TrajectoryDistanceLUT trajectoryDistanceLUT,
        Vec2d centerOfRotation,
        Vec2d turretPos,
        Vec2d goalPos,
        Vec2d robotLinearVel,
        Angle2d robotAngularVel,
        double currentExitSpeed,
        int tofEstimationIterations,
        boolean highArc
    ) {

        Vec2d robotToTurret = turretPos.minus(centerOfRotation);
        Vec2d robotToTurretPerp = robotToTurret.rotate(Angle2d.k90);

        Vec2d turretVel = robotToTurretPerp.times(robotAngularVel.radians()).plus(robotLinearVel);

        Vec2d turretToGoal = goalPos.minus(turretPos);
        double startDistFromGoal = turretToGoal.norm();

        // Calculate ideal trajectory (by impact angle)
        TrajectoryGoalInfo trajectoryGoalInfo = computeDisplacedGoal(
            trajectoryDistanceLUT,
            goalPos,
            turretPos,
            turretVel,
            startDistFromGoal,
            currentExitSpeed,
            tofEstimationIterations,
            highArc
        );

        Angle2d turretFieldAngle = trajectoryGoalInfo.displacedGoal.minus(turretPos).angle();

        return new TargetingInfo(
            trajectoryGoalInfo.targetTrajectory,
            trajectoryGoalInfo.compensatedTrajectory,
            trajectoryGoalInfo.displacedGoal,
            turretFieldAngle
        );
    }

    public static double[] calculateFilteredExitAngle(Trajectory target, Trajectory compensated) {
        double diff = Math.abs(compensated.exitSpeedMps - target.exitSpeedMps);
        double t = target.exitSpeedMOE <= 1e-9 ? 1.0 : Math.min(1, diff / target.exitSpeedMOE);
        double exitAngle = target.lerp(compensated, t).exitAngle.radians();
        return new double[] { exitAngle, t };
    }
}
