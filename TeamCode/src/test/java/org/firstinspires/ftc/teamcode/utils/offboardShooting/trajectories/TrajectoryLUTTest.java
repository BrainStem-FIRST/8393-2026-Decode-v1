package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

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
        trajectories.add(trajectory(5.0, 20.0, 30.0));
        trajectories.add(trajectory(7.5, 35.0, 45.0));
        trajectories.add(trajectory(10.0, 50.0, 60.0));

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
        double minAngle = trajectoryLUT.getMinExitAngleRad();
        assertNotNull(trajectoryLUT.getInterpolatedExitAngleTrajectory(minAngle));
    }

    @Test
    public void getInterpolatedExitAngleTrajectory_maxAngle_returnsNonNull() {
        double maxAngle = trajectoryLUT.getMaxExitAngleRad();
        assertNotNull(trajectoryLUT.getInterpolatedExitAngleTrajectory(maxAngle));
    }

    @Test
    public void exitAngleInRange_min_returnsTrue() {
        assertTrue(trajectoryLUT.exitAngleInRange(trajectoryLUT.getMinExitAngleRad()));
    }

    @Test
    public void exitAngleInRange_max_returnsTrue() {
        assertTrue(trajectoryLUT.exitAngleInRange(trajectoryLUT.getMaxExitAngleRad()));
    }

    private static Trajectory trajectory(double exitSpeedMps, double exitAngleDeg, double impactAngleDeg) {
        return new Trajectory(
                0.1,
                0.0,
                2.0,
                exitSpeedMps,
                Math.toRadians(exitAngleDeg),
                0.5,
                0.0,
                0.0,
                true
        );
    }
}
