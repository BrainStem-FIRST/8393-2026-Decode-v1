package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class TrajectoryLUTTest {
    private TrajectoryLUT trajectoryLUT;

    @Before
    public void setUp() {
        ArrayList<Trajectory> trajectories = new ArrayList<>();
        trajectories.add(trajectory(5.0, 20.0));
        trajectories.add(trajectory(7.5, 35.0));
        trajectories.add(trajectory(10.0, 50.0));

        trajectoryLUT = new TrajectoryLUT(
                1.0,
                2.0,
                0.1,
                0.0,
                0,
                trajectories
        );
    }

    @Test
    public void getInterpolatedExitAngleTrajectory_minAngle_returnsNonNull() {
        Angle2d minAngle = trajectoryLUT.getMinExitAngle();
        assertNotNull(trajectoryLUT.getInterpolatedExitAngleTrajectory(minAngle));
    }

    @Test
    public void getInterpolatedExitAngleTrajectory_maxAngle_returnsNonNull() {
        Angle2d maxAngle = trajectoryLUT.getMaxExitAngle();
        assertNotNull(trajectoryLUT.getInterpolatedExitAngleTrajectory(maxAngle));
    }


    private static Trajectory trajectory(double exitSpeedMps, double exitAngleDeg) {
        return new Trajectory(
                0.1,
                0.0,
                2.0,
                exitSpeedMps,
                Angle2d.fromDegrees(exitAngleDeg),
                0.5,
                0.0,
                0.0,
                true
        );
    }
}
