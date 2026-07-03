package org.firstinspires.ftc.teamcode.utils.offboardShooting.simpleCompensation;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.TrajectoryMath;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Vec2d;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryDistanceLUT;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryLUT;

import java.util.ArrayList;
import java.util.List;

public class SimpleLaunchCalculator {
    private final TrajectoryDistanceLUT trajectoryDistanceLUT;
    public SimpleLaunchCalculator() {
        //TODO: replace this example with your tuned values
        ArrayList<TrajectoryLUT> trajectoryLUTs = new ArrayList<>();

        TrajectoryLUT l1 = new TrajectoryLUT(1, 3, new ArrayList<>(List.of(
                new Trajectory(100, Angle2d.fromDegrees(45), 0),
                new Trajectory(110, Angle2d.fromDegrees(55), 0),
                new Trajectory(120, Angle2d.fromDegrees(65), 0),
                new Trajectory(130, Angle2d.fromDegrees(75), 0)
        )));
        TrajectoryLUT l2 = new TrajectoryLUT(2, 3, new ArrayList<>(List.of(
                new Trajectory(200, Angle2d.fromDegrees(45), 0),
                new Trajectory(210, Angle2d.fromDegrees(55), 0),
                new Trajectory(220, Angle2d.fromDegrees(65), 0),
                new Trajectory(230, Angle2d.fromDegrees(75), 0)
        )));
        TrajectoryLUT l3 = new TrajectoryLUT(3, 3, new ArrayList<>(List.of(
                new Trajectory(300, Angle2d.fromDegrees(45), 0),
                new Trajectory(310, Angle2d.fromDegrees(55), 0),
                new Trajectory(320, Angle2d.fromDegrees(65), 0),
                new Trajectory(330, Angle2d.fromDegrees(75), 0)
        )));

        trajectoryLUTs.add(l1);
        trajectoryLUTs.add(l2);
        trajectoryLUTs.add(l3);

        trajectoryDistanceLUT = TrajectoryDistanceLUT.fromTrajectoryLUTs(trajectoryLUTs);
    }


    private LaunchInfo getLaunchInfoNoSOTM(Vec2d exitPosition, Vec2d goalPosition, double shooterSpeed) {
        Vec2d exitPositionToGoal = goalPosition.minus(exitPosition);
        double distFromGoal = exitPositionToGoal.norm();

        Trajectory targetTrajectory = trajectoryDistanceLUT.getInterpolatedOptimalTrajectory(distFromGoal, true);
        Trajectory compensatedTrajectory = trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, shooterSpeed, true);

        double targetShooterSpeed = targetTrajectory.exitSpeedMps;
        Angle2d targetExitAngle = compensatedTrajectory.exitAngle;
        Angle2d targetFieldTurretAngle = exitPositionToGoal.angle();

        return new LaunchInfo(targetShooterSpeed, targetExitAngle, targetFieldTurretAngle);
    }

    private LaunchInfo getLaunchInfoSOTM(Vec2d centerOfRotation, Vec2d exitPosition, Vec2d centerOfRotationLinearVel, Angle2d angularVel, Vec2d goalPosition, double shooterSpeed) {
        TrajectoryMath.TargetingInfo targetingInfo = TrajectoryMath.calculateTargetingInfo(
                trajectoryDistanceLUT,
                centerOfRotation,
                exitPosition,
                goalPosition,
                centerOfRotationLinearVel,
                angularVel,
                shooterSpeed,
                5,
                true
        );

        return new LaunchInfo(
                targetingInfo.targetTrajectory().exitSpeedMps,
                targetingInfo.compensatedTrajectory().exitAngle,
                targetingInfo.turretFieldAngle()
        );
    }
}
