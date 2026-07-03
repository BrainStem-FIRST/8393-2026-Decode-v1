package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Vec3d;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class Trajectory {
    public final double dragCoeff;
    public final double magnusCoeff;
    public final double magnusPower;
    public final double exitSpeedMps;
    public final Angle2d exitAngle;
    public final double timeOfFlight;
    public final double exitSpeedMOE;
    public final Angle2d exitAngleMOE;
    public final boolean onTarget;

    public Trajectory(double exitSpeedMps, Angle2d exitAngle, double timeOfFlight) {
        this(0, 0, 0, exitSpeedMps, exitAngle, timeOfFlight, 0, Angle2d.kZero, true);
    }
    public Trajectory(
            double dragCoeff,
            double magnusCoeff,
            double magnusPower,
            double exitSpeedMps,
            Angle2d exitAngle,
            double timeOfFlight,
            double exitSpeedMOE,
            Angle2d exitAngleMOE,
            boolean onTarget) {
        this.dragCoeff = dragCoeff;
        this.magnusCoeff = magnusCoeff;
        this.magnusPower = magnusPower;
        this.exitSpeedMps = exitSpeedMps;
        this.exitAngle = exitAngle;
        this.timeOfFlight = timeOfFlight;
        this.exitSpeedMOE = exitSpeedMOE;
        this.exitAngleMOE = exitAngleMOE;
        this.onTarget = onTarget;
    }

    public Trajectory lerp(Trajectory other, double t) {
        double interpLaunchSpeed = lerp(exitSpeedMps, other.exitSpeedMps, t);
        Angle2d interpExitAngle = exitAngle.lerp(other.exitAngle, t);
        double interpTOF = lerp(timeOfFlight, other.timeOfFlight, t);
        double interpSpeedMoe = lerp(exitSpeedMOE, other.exitSpeedMOE, t);
        Angle2d interpAngleMoe = exitAngleMOE.lerp(other.exitAngleMOE, t);

        return new Trajectory(
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
    }


    protected Trajectory invalidate() {
        return new Trajectory(
                dragCoeff,
                magnusCoeff,
                magnusPower,
                exitSpeedMps,
                exitAngle,
                timeOfFlight,
                exitSpeedMOE,
                exitAngleMOE,
                false
        );
    }

    protected double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    @NonNull
    @Override
    public String toString() {
        DecimalFormat df = new DecimalFormat("#.###");
        DecimalFormat df2 = new DecimalFormat("#.######");
        return "ExitSpeed" + df2.format(exitSpeedMps) + "m/s | ExitAngle: " + df.format(exitAngle.degrees()) + "deg | SpeedMOE: " + df.format(exitSpeedMOE) + "mps | AngleMOE: " + df.format(exitAngleMOE.degrees()) + "deg | ToF: " + df.format(timeOfFlight) + "s";
    }
    public String toStringShort() {
        DecimalFormat df = new DecimalFormat("#.###");
        DecimalFormat df2 = new DecimalFormat("#.######");
        return df2.format(exitSpeedMps) + "m/s " + df.format(exitAngle.degrees()) + "deg";
    }

    public ArrayList<Vec3d> simulateTrajectory(
            int numPoints,
            int sparsityForTelemetry,
            Angle2d turretAngle,
            Vec3d startPosition,
            Vec3d startVelocity) {

        ArrayList<Vec3d> points = new ArrayList<>();
        if (numPoints <= 2)
            throw new IllegalArgumentException("numPoints must be at least 3");

        if (sparsityForTelemetry <= 0)
            throw new IllegalArgumentException("sparsityForTelemetry must be positive");

        double dt = timeOfFlight / (numPoints - 1);

        double x = startPosition.x();
        double y = startPosition.y();
        double z = startPosition.z();

        double launchHorizontalSpeed = exitSpeedMps * exitAngle.cos();
        double launchVerticalSpeed = exitSpeedMps * exitAngle.sin();

        double launchVx = launchHorizontalSpeed * turretAngle.cos();
        double launchVy = launchHorizontalSpeed * turretAngle.sin();
        double launchVz = launchVerticalSpeed;

        double vx = launchVx + startVelocity.x();
        double vy = launchVy + startVelocity.y();
        double vz = launchVz + startVelocity.z();

        /*
         * Horizontal spin axis.
         *
         * For turretFieldAngle = 0:
         * forward = +x
         * spinAxis = +y
         *
         * cross(velocity, spinAxis) = +z, so positive magnusCoef gives upward lift.
         * Negative magnusCoef flips the direction, representing topspin.
         */
        double spinAxisX = -turretAngle.sin();
        double spinAxisY = turretAngle.cos();
        double spinAxisZ = 0.0;

        points.add(new Vec3d(x, y, z));

        for (int i = 1; i < numPoints; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);

            double azGravity = -9.81;

            double axDrag = 0.0;
            double ayDrag = 0.0;
            double azDrag = 0.0;

            double axMagnus = 0.0;
            double ayMagnus = 0.0;
            double azMagnus = 0.0;

            if (speed > 1e-9) {
                /*
                 * Drag acceleration:
                 * a_drag = -dragCoef * |v| * v
                 *
                 * This is equivalent to speed^2 drag because:
                 * |v| * v has magnitude |v|^2.
                 */
                axDrag = -dragCoeff * speed * vx;
                ayDrag = -dragCoeff * speed * vy;
                azDrag = -dragCoeff * speed * vz;

                /*
                 * Magnus direction:
                 * perpendicular to velocity and spin axis.
                 *
                 * direction = normalize(cross(velocity, spinAxis))
                 */
                double magnusDirX = vy * spinAxisZ - vz * spinAxisY;
                double magnusDirY = vz * spinAxisX - vx * spinAxisZ;
                double magnusDirZ = vx * spinAxisY - vy * spinAxisX;

                double magnusDirMag = Math.sqrt(
                        magnusDirX * magnusDirX +
                                magnusDirY * magnusDirY +
                                magnusDirZ * magnusDirZ
                );

                if (magnusDirMag > 1e-9) {
                    magnusDirX /= magnusDirMag;
                    magnusDirY /= magnusDirMag;
                    magnusDirZ /= magnusDirMag;

                    /*
                     * Magnus acceleration magnitude:
                     * a_magnus = magnusCoef * speed^magnusPower
                     *
                     * magnusCoef > 0: backspin
                     * magnusCoef < 0: topspin
                     */
                    double magnusAccel = magnusCoeff * Math.pow(speed, magnusPower);

                    axMagnus = magnusDirX * magnusAccel;
                    ayMagnus = magnusDirY * magnusAccel;
                    azMagnus = magnusDirZ * magnusAccel;
                }
            }

            double ax = axDrag + axMagnus;
            double ay = ayDrag + ayMagnus;
            double az = azGravity + azDrag + azMagnus;

            vx += ax * dt;
            vy += ay * dt;
            vz += az * dt;

            x += vx * dt;
            y += vy * dt;
            z += vz * dt;

            points.add(new Vec3d(x, y, z));
        }

        ArrayList<Vec3d> pointsToPublish = IntStream.range(0, points.size())
                .filter(i -> i % sparsityForTelemetry == 0)
                .mapToObj(points::get)
                .collect(Collectors.toCollection(ArrayList::new));

        if (!pointsToPublish.get(pointsToPublish.size() - 1).equals(points.get(points.size() - 1))) {
            pointsToPublish.add(points.get(points.size() - 1));
        }

        return pointsToPublish;
    }

}
