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
        Trajectory idealTargetTrajectory,
        Trajectory actualTargetTrajectory,
        Vec2d displacedGoal,
        Angle2d idealTurretFieldAngle,
        Angle2d actualTurretFieldAngle
    ) {}

    /**
     * Computes the displaced goal and the last trajectory after several iterations,
     * updating based on the provided function for next trajectory lookup.
     */
    private static TrajectoryGoalInfo computeDisplacedGoal(
        TrajectoryDistanceLUT trajectoryLUT,
        Vec2d goalPos,
        Vec2d turretPos,
        Vec2d turretVel,
        double startDistFromGoalM,
        double speed,
        int tofEstimationIterations,
        boolean useOptimalTrajectory // true = optimal trajectory lookup, false = exit-speed lookup
    ) {
        Vec2d displacedGoal = goalPos;
        Vec2d turretToGoal;
        double distFromGoal = startDistFromGoalM;
        Trajectory optimalTrajectory = trajectoryLUT.getInterpolatedOptimalTrajectory(distFromGoal);
        Trajectory trajectory = useOptimalTrajectory
            ? optimalTrajectory
            : trajectoryLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, speed, optimalTrajectory.exitAngle);

        if (trajectory == null) return new TrajectoryGoalInfo(null, null, 0.0);

        for (int i = 0; i < tofEstimationIterations; i++) {
            displacedGoal = goalPos.minus(turretVel.times(trajectory.timeOfFlight * 0.85));
            turretToGoal = displacedGoal.minus(turretPos);
            distFromGoal = turretToGoal.norm();

            optimalTrajectory = trajectoryLUT.getInterpolatedOptimalTrajectory(distFromGoal);
            trajectory = useOptimalTrajectory
                ? optimalTrajectory
                : trajectoryLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, speed, optimalTrajectory.exitAngle);

            if (trajectory == null) return new TrajectoryGoalInfo(null, null, 0.0);
        }
        return new TrajectoryGoalInfo(trajectory, displacedGoal, distFromGoal);
    }

    private static class TrajectoryGoalInfo {
        final Trajectory trajectory;
        final Vec2d displacedGoal;
        final double distFromGoal;
        TrajectoryGoalInfo(Trajectory t, Vec2d g, double d) {
            this.trajectory = t;
            this.displacedGoal = g;
            this.distFromGoal = d;
        }
    }

    // TAKES EVERYTHING IN METERS
    public static TargetingInfo calculateTargetingInfo(
        TrajectoryDistanceLUT trajectoryLUT,
        Vec2d centerOfRotation,
        Vec2d turretPos,
        Vec2d goalPos,
        Vec2d robotLinearVel,
        double robotAngularVelRad,
        double currentExitSpeed,
        int tofEstimationIterations
    ) {

        Vec2d robotToTurret = turretPos.minus(centerOfRotation);
        Vec2d robotToTurretPerp = new Vec2d(-robotToTurret.y(), robotToTurret.x()*1);

        Vec2d turretVel = robotToTurretPerp.times(robotAngularVelRad).plus(robotLinearVel);

        Vec2d turretToGoal = goalPos.minus(turretPos);
        double startDistFromGoal = Math.hypot(turretToGoal.x(), turretToGoal.y());

        // Calculate ideal trajectory (by impact angle)
        TrajectoryGoalInfo idealInfo = computeDisplacedGoal(
            trajectoryLUT,
            goalPos,
            turretPos,
            turretVel,
            startDistFromGoal,
            currentExitSpeed,
            tofEstimationIterations,
            true);

        if (idealInfo.trajectory == null) return null;

        Angle2d idealTurretFieldAngle = Angle2d.fromRadians(Math.atan2(
            idealInfo.displacedGoal.minus(turretPos).y(),
            idealInfo.displacedGoal.minus(turretPos).x()
        ));

        // Calculate actual trajectory (by exit speed)
        TrajectoryGoalInfo actualInfo = computeDisplacedGoal(
            trajectoryLUT,
            goalPos,
            turretPos,
            turretVel,
            idealInfo.distFromGoal,
            currentExitSpeed,
            tofEstimationIterations,
            false
        );

        Angle2d actualTurretFieldAngle;
        if (actualInfo.trajectory == null)
            actualTurretFieldAngle = null;
        else
            actualTurretFieldAngle = Angle2d.fromRadians(Math.atan2(
                actualInfo.displacedGoal.minus(turretPos).y(),
                actualInfo.displacedGoal.minus(turretPos).x()
            ));

        return new TargetingInfo(
            idealInfo.trajectory,
            actualInfo.trajectory,
            actualInfo.displacedGoal, // return displacedGoal from actual/exitSpeed
            idealTurretFieldAngle,
            actualTurretFieldAngle
        );
    }

    public static double[] calculateFilteredExitAngle(Trajectory target, Trajectory compensated) {
        double diff = Math.abs(compensated.exitSpeedMps - target.exitSpeedMps);
        double t = target.exitSpeedMOE <= 1e-9 ? 1.0 : Math.min(1, diff / target.exitSpeedMOE);
        double exitAngle = target.lerp(compensated, t).exitAngle.radians();
        return new double[] { exitAngle, t };
    }
}
