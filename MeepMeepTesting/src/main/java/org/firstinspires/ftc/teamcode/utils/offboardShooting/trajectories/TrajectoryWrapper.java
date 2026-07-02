package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;

public class TrajectoryWrapper extends Trajectory {
    public final TrajectoryType trajectoryType;
    public TrajectoryWrapper(Trajectory trajectory, TrajectoryType trajectoryType) {
        super(trajectory.dragCoeff, trajectory.magnusCoeff, trajectory.magnusPower, trajectory.exitSpeedMps, trajectory.exitAngle, trajectory.timeOfFlight, trajectory.exitSpeedMOE, trajectory.exitAngleMOERad, trajectory.onTarget);
        this.trajectoryType = trajectoryType;
    }
    public TrajectoryWrapper lerp(TrajectoryWrapper other, double t) {
        double interpLaunchSpeed = lerp(exitSpeedMps, other.exitSpeedMps, t);
        Angle2d interpExitAngle = exitAngle.lerp(other.exitAngle, t);
        double interpTOF = lerp(timeOfFlight, other.timeOfFlight, t);
        double interpSpeedMoe = lerp(exitSpeedMOE, other.exitSpeedMOE, t);
        double interpAngleMoe = lerp(exitAngleMOERad, other.exitAngleMOERad, t);

        Trajectory lerpedTrajectory = new Trajectory(
                dragCoeff,
                magnusCoeff,
                magnusPower,
                interpLaunchSpeed,
                interpExitAngle,
                interpTOF,
                interpSpeedMoe,
                interpAngleMoe,
                onTarget && other.onTarget
        );
        TrajectoryType type;
        if (t == 0)
            type = trajectoryType;
        else if (t == 1)
            type = other.trajectoryType;
        else {
            if ((trajectoryType == TrajectoryType.LOW_ARC && other.trajectoryType == TrajectoryType.HIGH_ARC) ||
                    trajectoryType == TrajectoryType.MIXED_ARCS || other.trajectoryType == TrajectoryType.MIXED_ARCS)
                type = TrajectoryType.MIXED_ARCS;
            else if (trajectoryType == TrajectoryType.LOWEST_SPEED)
                type = other.trajectoryType;
            else
                type = trajectoryType;
        }

        return new TrajectoryWrapper(lerpedTrajectory, type);
    }

    @Override
    public TrajectoryWrapper invalidate() {
        return new TrajectoryWrapper(super.invalidate(), trajectoryType);
    }
}
