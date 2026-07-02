package org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Vector3d;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class Trajectory {
    public final double dragCoeff;
    public final double magnusCoeff;
    public final double magnusPower;
    public final double exitSpeedMps;
    public final double exitAngleRad;
    public final double timeOfFlight;
    public final double exitSpeedMOE;
    public final double exitAngleMOERad;
    public final boolean onTarget;

    public Trajectory(
            double dragCoeff,
            double magnusCoeff,
            double magnusPower,
            double launchSpeedMps,
            double exitAngleRad,
            double timeOfFlight,
            double exitSpeedMOE,
            double exitAngleMOERad,
            boolean onTarget) {
        this.dragCoeff = dragCoeff;
        this.magnusCoeff = magnusCoeff;
        this.magnusPower = magnusPower;
        this.exitSpeedMps = launchSpeedMps;
        this.exitAngleRad = exitAngleRad;
        this.timeOfFlight = timeOfFlight;
        this.exitSpeedMOE = exitSpeedMOE;
        this.exitAngleMOERad = exitAngleMOERad;
        this.onTarget = onTarget;
    }

    public Trajectory lerp(Trajectory other, double t) {
        double interpLaunchSpeed = lerp(exitSpeedMps, other.exitSpeedMps, t);
        double interpExitAngle = lerp(exitAngleRad, other.exitAngleRad, t);
        double interpTOF = lerp(timeOfFlight, other.timeOfFlight, t);
        double interpSpeedMoe = lerp(exitSpeedMOE, other.exitSpeedMOE, t);
        double interpAngleMoe = lerp(exitAngleMOERad, other.exitAngleMOERad, t);

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
                exitAngleRad,
                timeOfFlight,
                exitSpeedMOE,
                exitAngleMOERad,
                false
        );
    }

    protected double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    @NonNull
    @Override
    public String toString() {
        DecimalFormat df = new DecimalFormat("0.000");
        return "ExitSpeed" + df.format(exitSpeedMps) + "m/s | ExitAngle: " + df.format(Math.toDegrees(exitAngleRad)) + "deg | SpeedMOE: " + df.format(exitSpeedMOE) + "mps | AngleMOE: " + df.format(Math.toDegrees(exitAngleMOERad)) + "deg | ToF: " + df.format(timeOfFlight) + "s";
    }
    public String toStringShort() {
        DecimalFormat df = new DecimalFormat("0.000");
        return df.format(exitSpeedMps) + "m/s " + df.format(Math.toDegrees(exitAngleRad)) + "deg";
    }

    public ArrayList<Vector3d> simulateTrajectory(
            int numPoints,
            int sparsityForTelemetry,
            double turretAngleRad,
            Vector3d startPosition,
            Vector3d startVelocity) {

        ArrayList<Vector3d> points = new ArrayList<>();
        if (numPoints <= 2)
            throw new IllegalArgumentException("numPoints must be at least 3");

        if (sparsityForTelemetry <= 0)
            throw new IllegalArgumentException("sparsityForTelemetry must be positive");

        double dt = timeOfFlight / (numPoints - 1);

        double x = startPosition.x();
        double y = startPosition.y();
        double z = startPosition.z();

        double launchHorizontalSpeed = exitSpeedMps * Math.cos(exitAngleRad);
        double launchVerticalSpeed = exitSpeedMps * Math.sin(exitAngleRad);

        double launchVx = launchHorizontalSpeed * Math.cos(turretAngleRad);
        double launchVy = launchHorizontalSpeed * Math.sin(turretAngleRad);
        double launchVz = launchVerticalSpeed;

        double vx = launchVx + startVelocity.x();
        double vy = launchVy + startVelocity.y();
        double vz = launchVz + startVelocity.z();

        /*
         * Horizontal spin axis.
         *
         * For turretAngleRad = 0:
         * forward = +x
         * spinAxis = +y
         *
         * cross(velocity, spinAxis) = +z, so positive magnusCoef gives upward lift.
         * Negative magnusCoef flips the direction, representing topspin.
         */
        double spinAxisX = -Math.sin(turretAngleRad);
        double spinAxisY = Math.cos(turretAngleRad);
        double spinAxisZ = 0.0;

        points.add(new Vector3d(x, y, z));

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

            points.add(new Vector3d(x, y, z));
        }

        ArrayList<Vector3d> pointsToPublish = IntStream.range(0, points.size())
                .filter(i -> i % sparsityForTelemetry == 0)
                .mapToObj(points::get)
                .collect(Collectors.toCollection(ArrayList::new));

        if (!pointsToPublish.get(pointsToPublish.size() - 1).equals(points.get(points.size() - 1))) {
            pointsToPublish.add(points.get(points.size() - 1));
        }

        return pointsToPublish;
    }

}