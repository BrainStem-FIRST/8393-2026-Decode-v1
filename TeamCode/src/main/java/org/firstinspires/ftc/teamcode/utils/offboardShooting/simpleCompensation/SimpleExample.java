package org.firstinspires.ftc.teamcode.utils.offboardShooting.simpleCompensation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.TrajectoryMath;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Vec2d;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryDistanceLUT;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryLUT;

import java.util.ArrayList;
import java.util.List;

public class SimpleExample extends OpMode {

    private TrajectoryDistanceLUT trajectoryDistanceLUT;
    @Override
    public void init() {
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

    @Override
    public void loop() {
    }
    
    private LaunchInfo noSotmExample() {
        Vec2d exitPositionInches = Vec2d.kZero;
        Vec2d goalPositionInches = new Vec2d(62, 62);
        double shooterSpeed = 200;

        Vec2d exitPositionToGoal = goalPositionInches.minus(exitPositionInches);
        double distFromGoalInches = exitPositionToGoal.norm();

        Trajectory targetTrajectory = trajectoryDistanceLUT.getInterpolatedOptimalTrajectory(distFromGoalInches, true);
        Trajectory compensatedTrajectory = trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(distFromGoalInches, shooterSpeed, true);

        double targetShooterSpeed = targetTrajectory.exitSpeedMps;
        Angle2d targetExitAngle = compensatedTrajectory.exitAngle;
        Angle2d targetFieldTurretAngle = exitPositionToGoal.angle();
        
        return new LaunchInfo(targetShooterSpeed, targetExitAngle, targetFieldTurretAngle);
    }
    
    private LaunchInfo sotmExample() {
        Vec2d centerOfRotation = Vec2d.kZero;
        Vec2d exitPositionInches = new Vec2d(0, 0.5);
        Vec2d robotLinearVel = Vec2d.kZero;
        Angle2d robotAngularVel = Angle2d.kZero;
        
        Vec2d goalPositionInches = new Vec2d(62, 62);
        double shooterSpeed = 200;

        TrajectoryMath.TargetingInfo targetingInfo = TrajectoryMath.calculateTargetingInfo(
                trajectoryDistanceLUT,
                centerOfRotation,
                exitPositionInches,
                goalPositionInches,
                robotLinearVel,
                robotAngularVel,
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
